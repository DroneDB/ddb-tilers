#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Run a benchmark suite: backends x datasets x scenarios x threads x formats x cache.

Reads bench/suites/<suite>.toml (items, matrix, scenario families, [item.X] skip flags) and
narrows the matrix with bench/harness/profiles.toml. For every case the
backends run interleaved ABBA (rep 0: A B, rep 1: B A, ...), each repetition in a fresh
ddbt-bench process.

Tiles   ddbt-bench tile on pinned workers; cold = page cache evicted (inputs and DDB_HOME
        artefacts), 1 round, no warmup; warm = inputs read into the page cache, warmup rounds
        then measured rounds. One fresh DDB_HOME per case and backend; inputs that are not
        COGs are converted by the backend on the first tile: that is timed once per case as
        "prepare" (never inside the tile latencies). Raw LAS/LAZ are converted once to COPC
        with the baseline (cached under work/prepared/) and both backends tile that COPC.
Builds  ddbt-bench run (process tree time/RSS/I/O) of `ddb cog|copc` (baseline) or
        `ddbt cog|copc build`, CPU affinity = the pin set of the thread count, DDB_BUILD_THREADS;
        always cold; outputs validated (validate_cog.py / validate_copc.py) and deleted unless
        --keep. Disk guard: free space >= estimate + 20 GB.
Gates   quiet (load1 < 1 and no process > 10% CPU over 2 s) before each case, thermal
        (Tctl < 55 C) before each invocation, swap (pswpin/pswpout delta > 0 marks the
        repetition invalid; it is rerun up to --max-retries times).
Guard   (path, size, mtime) of every input folder before and after each case: any change
        aborts the run (exit 3).

Results: <work>/results/<UTC>-<sha7>-<suite>/run.json (schema ddbt-bench/1) + report.md, raw
driver outputs under raw/, and a summary copy (no raw data) under bench/results/<run-id>/.
"""
import argparse
import hashlib
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
import tomllib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common  # noqa: E402
import gates  # noqa: E402
import requests as reqgen  # noqa: E402
import stats  # noqa: E402

SCENARIO_ALIASES = {
    "sweep-per-zoom": "sweep", "sweep": "sweep",
    "pan-walk-1920x1080": "pan", "pan": "pan",
    "retina-512": "retina", "retina": "retina",
    "edge-tiles": "edge", "edge": "edge",
}
BUILD_SCENARIOS = ("cog", "copc-untwine", "copc-pdal")
SCENARIO_SIZE = {"sweep": 256, "pan": 256, "edge": 256, "retina": 512}
PERF_EVENTS = "cycles,instructions,cache-misses,branch-misses"

TILE_METRICS = ["p50_ms", "p95_ms", "p99_ms", "tiles_per_s", "rss_peak_kb"]
BUILD_METRICS = ["wall_s", "cpu_s", "tree_rss_peak_kb", "write_bytes"]
PREPARED_ROOT = os.path.join(common.WORK, "prepared")


# --------------------------------------------------------------------------- configuration

class Backend:
    def __init__(self, name, kind, lib=None, cli=None, env_file=None):
        if kind not in ("legacy", "ddbt"):
            raise SystemExit("backend %s: kind must be legacy or ddbt" % name)
        if env_file and not os.path.isfile(env_file):
            raise SystemExit("backend %s: no env file %s" % (name, env_file))
        self.name, self.kind, self.lib, self.cli, self.env_file = name, kind, lib, cli, env_file
        if kind == "legacy":
            self.lib = self.lib or common.baseline_lib()
            self.cli = self.cli or os.path.join(common.BASELINE_DIR, "usr", "bin", "ddb")
        else:
            self.lib = self.lib or common.default_ddbt_lib()
            self.cli = self.cli or common.default_ddbt_cli()

    def describe(self):
        d = {"kind": self.kind, "lib": self.lib, "cli": self.cli}
        if self.env_file:
            d["env_file"] = self.env_file
        if self.lib and os.path.isfile(self.lib):
            d["lib_size"] = os.path.getsize(self.lib)
            d["lib_sha256"] = sha256_file(self.lib)
        return d


def parse_backend(spec):
    """name=kind:lib[:cli[:env]] | kind (legacy/ddbt with default paths) | name=kind.

    env: a file sourced (bash) for the runs of this backend instead of the suite's baseline env
    (legacy) or the current environment (ddbt), e.g. a snapshot of another libddb build
    (scripts/snapshot-build.sh)."""
    if "=" not in spec:
        return Backend(spec, spec)
    name, rest = spec.split("=", 1)
    parts = rest.split(":")
    part = lambda i: parts[i] if len(parts) > i and parts[i] else None
    return Backend(name, parts[0], part(1), part(2), part(3))


def sha256_file(path, limit=None):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load_suite(name_or_path):
    path = name_or_path
    if not os.path.isfile(path):
        path = os.path.join(common.SUITES_DIR, name_or_path + ".toml")
    with open(path, "rb") as f:
        s = tomllib.load(f)
    meta = s.get("suite", {})
    suite = {
        "name": meta.get("name") or s.get("name") or os.path.splitext(os.path.basename(path))[0],
        "file": path,
        "manifest": meta.get("corpus_manifest") or s.get("corpus_manifest") or common.MANIFEST,
        "items": list(meta.get("items") or s.get("items") or s.get("datasets") or []),
        "matrix": dict(s.get("matrix", {})),
        "tile_scenarios": [],
        "build_scenarios": [],
        "item_flags": {k: dict(v) for k, v in (s.get("item") or {}).items()},
        "baseline": dict(s.get("baseline", {})),
    }
    sc = s.get("scenarios", {})
    for t in sc.get("tile_requests", sc.get("tiles", ["sweep", "pan", "retina", "edge"])):
        if t in SCENARIO_ALIASES:
            suite["tile_scenarios"].append(SCENARIO_ALIASES[t])
        else:
            common.log("suite: unknown tile scenario %r ignored" % t)
    for b in sc.get("builds", list(BUILD_SCENARIOS)):
        if b in BUILD_SCENARIOS:
            suite["build_scenarios"].append(b)
        else:
            common.log("suite: unknown build scenario %r ignored" % b)
    return suite


def load_profile(suite_name, path=os.path.join(common.HERE, "profiles.toml")):
    with open(path, "rb") as f:
        p = tomllib.load(f)
    base = p.get("default", {})
    over = p.get(suite_name, {})
    prof = {k: v for k, v in base.items() if not isinstance(v, dict)}
    prof.update({k: v for k, v in over.items() if not isinstance(v, dict)})
    prof["tile"] = {}
    for scen in ("sweep", "pan", "retina", "edge"):
        prof["tile"][scen] = dict(base.get("tile", {}).get(scen, {}), **over.get("tile", {}).get(scen, {}))
    prof["build"] = dict(base.get("build", {}), **over.get("build", {}))
    return prof


def narrow(profile_list, suite_list, cli_list):
    if cli_list:
        return list(cli_list)
    if profile_list is None:
        return list(suite_list or [])
    if not suite_list:
        return list(profile_list)
    return [v for v in profile_list if v in suite_list]


def slug(s):
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", s)


def csv_list(s, conv=str):
    return [conv(x) for x in s.split(",") if x.strip()] if s else None


# --------------------------------------------------------------------------- read-only guard

def snapshot_dirs(paths):
    """{path: (size, mtime_ns)} of each input and of every entry of its folder (non-recursive)."""
    snap = {}
    for p in paths:
        # inside a .ddb build tree, watch the whole tree of the dataset (a sidecar may appear anywhere in it)
        ap_ = os.path.abspath(p)
        if os.sep + ".ddb" + os.sep in ap_:
            root = ap_[:ap_.index(os.sep + ".ddb" + os.sep)]
            for r, ds, fs in os.walk(root):
                for n in ds + fs:
                    fp_ = os.path.join(r, n)
                    try:
                        st = os.lstat(fp_)
                        snap[fp_] = (st.st_size, st.st_mtime_ns)
                    except OSError:
                        snap[fp_] = None
        d = os.path.dirname(os.path.abspath(p))
        try:
            with os.scandir(d) as it:
                for e in it:
                    try:
                        st = e.stat(follow_symlinks=False)
                        snap[e.path] = (st.st_size, st.st_mtime_ns)
                    except OSError:
                        snap[e.path] = None
        except OSError:
            pass
        try:
            st = os.stat(p)
            snap[os.path.abspath(p)] = (st.st_size, st.st_mtime_ns)
        except OSError:
            snap[os.path.abspath(p)] = None
    return snap


def snapshot_diff(a, b):
    out = []
    for k in sorted(set(a) | set(b)):
        if a.get(k) != b.get(k):
            out.append({"path": k, "before": a.get(k), "after": b.get(k)})
    return out


# --------------------------------------------------------------------------- perf

def perf_wrap(cmd, out_file):
    return ["perf", "stat", "-x,", "-o", out_file, "-e", PERF_EVENTS, "--"] + cmd


def parse_perf(path):
    res = {}
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split(",")
                if len(parts) < 3:
                    continue
                ev = parts[2].split(":")[0].replace("-", "_")
                try:
                    res[ev] = int(float(parts[0]))
                except ValueError:
                    res[ev] = None  # <not counted> / <not supported>
    except OSError:
        pass
    return res


# --------------------------------------------------------------------------- runner

class Runner:
    def __init__(self, a):
        self.a = a
        self.driver = common.find_driver(a.driver)
        self.suite = load_suite(a.suite)
        self.profile = load_profile(a.profile or self.suite["name"])
        self.manifest_path = a.manifest or self.suite["manifest"]
        self.items, _ = common.load_manifest(self.manifest_path)
        specs = a.backend or ["legacy"]
        if a.aa:
            b0 = parse_backend(specs[0])
            self.backends = [b0, Backend(b0.name + "-A2", b0.kind, b0.lib, b0.cli)]
        else:
            self.backends = [parse_backend(s) for s in specs]
        names = [b.name for b in self.backends]
        if len(set(names)) != len(names):
            raise SystemExit("backend names must be unique: %s" % names)
        for b in self.backends:
            if not b.lib or not os.path.isfile(b.lib):
                raise SystemExit("backend %s: library not found (%s)" % (b.name, b.lib))
        self.base = a.base or self.backends[0].name
        self.sha7 = common.git_sha(short=True)
        self.run_id = a.run_id or "%s-%s-%s" % (common.utc_stamp(), self.sha7, self.suite["name"])
        self.results_dir = os.path.join(a.results_root or common.RESULTS_ROOT, self.run_id)
        self.raw_dir = os.path.join(self.results_dir, "raw")
        self.req_dir = os.path.join(self.results_dir, "requests")
        self.home_root = os.path.join(common.HOME_ROOT, self.run_id)
        self.env_file = self.suite["baseline"].get("env_file") or common.BASELINE_ENV
        self.legacy_env = common.baseline_env(self.env_file)
        self.reps = a.reps or int(self.profile.get("reps", 5))
        self.sysinfo, _ = common.driver_json(self.driver, ["sysinfo"])
        self.pins = common.pin_sets(self.sysinfo)
        for alias in a.pin_set or []:
            threads, name = alias.split("=", 1)
            if name not in self.pins:
                raise SystemExit("--pin-set: unknown pin set %s (known: %s)" % (name, ", ".join(sorted(self.pins))))
            self.pins[threads] = self.pins[name]
        self.cases = []
        self.events = []
        self.tile_paths = dict(kv.split("=", 1) for kv in (a.tile_path or []))
        self.gate_cpus = gates.parse_cpulist(a.gate_cpus) if a.gate_cpus else None
        self.prepare_seen = {}  # (item, backend) -> the real prepare measurement the seeded cases refer to
        self.iso_samples = []   # per quiet gate and per invocation: utilisation of the gate CPUs / the other CPUs
        self.temps = []
        self.load_start = gates.loadavg()

    # ---- environment
    def env_for(self, backend, home, extra=None):
        if backend.env_file:
            env = self.backend_env(backend.env_file)
        elif backend.kind == "legacy":
            env = dict(self.legacy_env)
        else:
            env = {k: v for k, v in os.environ.items() if not k.startswith("DDB_")}
            for kv in self.a.ddbt_env or []:
                k, _, v = kv.partition("=")
                env[k] = v
        env["GDAL_PAM_ENABLED"] = "NO"
        env["DDB_HOME"] = home
        env.update(extra or {})
        return env

    def backend_env(self, env_file):
        """The environment of a backend's own env file, sourced once per run."""
        cache = self.__dict__.setdefault("_backend_envs", {})
        if env_file not in cache:
            cache[env_file] = common.baseline_env(env_file)
        return dict(cache[env_file])

    def fresh_dir(self, path):
        """Create an empty directory below one of our own roots (never elsewhere)."""
        ap = os.path.abspath(path)
        if not any(ap.startswith(os.path.abspath(r) + os.sep) for r in (common.HOME_ROOT, common.OUT_ROOT)):
            raise RuntimeError("refusing to reset %s (outside the work roots)" % ap)
        if os.path.isdir(ap):
            shutil.rmtree(ap)
        os.makedirs(ap)
        return ap

    def remove_dir(self, path):
        ap = os.path.abspath(path)
        if any(ap.startswith(os.path.abspath(r) + os.sep) for r in (common.HOME_ROOT, common.OUT_ROOT)):
            shutil.rmtree(ap, ignore_errors=True)

    # ---- gates
    def quiet_gate(self):
        if self.a.gate_policy == "off":
            return {"policy": "off"}
        ok, s, waited = gates.wait_quiet(self.a.quiet_timeout, self.a.load_max, self.a.proc_max_pct,
                                         gate_cpus=self.gate_cpus)
        res = {"ok": ok, "load1": s["load1"], "busy": s["busy"][:3], "waited_s": round(waited, 1)}
        if self.gate_cpus:
            res["gate_cpus"] = common.cpulist(self.gate_cpus)
            res["gate_cpus_util_pct"] = s["gate_cpus_util_pct"]
            res["other_cpus_util_pct"] = s["other_cpus_util_pct"]
            self.iso_samples.append({"where": "quiet-gate", "other": s["other_cpus_util_pct"],
                                     "gate": s["gate_cpus_util_pct"], "load1": s["load1"]})
        if not ok:
            msg = "quiet gate failed after %.0f s: load1 %.2f, busy %s%s" % (
                waited, s["load1"], s["busy"][:3], " (gate cpus %s)" % common.cpulist(self.gate_cpus)
                if self.gate_cpus else "")
            if self.a.gate_policy == "strict":
                raise SystemExit(msg)
            common.log("WARNING " + msg)
        return res

    def thermal_gate(self):
        if self.a.gate_policy == "off":
            return {"policy": "off"}
        ok, t, waited = gates.wait_thermal(self.a.thermal_max, self.a.thermal_timeout)
        if not ok:
            msg = "thermal gate: %.1f C after %.0f s" % (t, waited)
            if self.a.gate_policy == "strict":
                raise SystemExit(msg)
            common.log("WARNING " + msg)
        self.temps.append(t)
        return {"ok": ok, "temp_c": t, "waited_s": round(waited, 1)}

    # ---- page cache
    def pagecache(self, action, paths):
        paths = [p for p in paths if p and os.path.exists(p)]
        if not paths:
            return None
        d, rc = common.driver_json(self.driver, ["pagecache", action] + paths, check=False)
        tot = d.get("total", {})
        if action == "evict" and rc != 0:
            common.log("WARNING evict left %.1f%% resident" % tot.get("resident_pct", -1))
        return {"action": action, "resident_pct": tot.get("resident_pct"), "files": tot.get("files")}

    # ---- planning
    def tile_input(self, item):
        """(path to tile, needs_prepare). Raw point clouds are converted once to COPC."""
        probe = item.get("probe") or {}
        if item["id"] in self.tile_paths:
            return self.tile_paths[item["id"]], False
        if item.get("kind") == "pointcloud":
            if probe.get("copc") or item["path"].endswith(".copc.laz"):
                return item["path"], False
            return self.prepared_copc(item), False
        # libddb tiles a COG in place only when it is already in EPSG:3857 (R7 and R9 are COGs in UTM: it rebuilds them)
        in_place = probe.get("is_cog", False) and (probe.get("crs") or {}).get("code") == "EPSG:3857"
        return item["path"], not in_place

    def prepared_copc(self, item):
        key = "%s-%s" % (item["id"], (item.get("sha256") or "nosha")[:12])
        out_dir = os.path.join(PREPARED_ROOT, key)
        out = os.path.join(out_dir, "cloud.copc.laz")
        if self.a.dry_run or os.path.isfile(out):
            return out
        common.log("preparing COPC of %s with the baseline (Untwine) into %s" % (item["id"], out_dir))
        os.makedirs(out_dir, exist_ok=True)
        home = self.fresh_dir(os.path.join(self.home_root, "prepare", item["id"]))
        env = self.env_for(Backend("legacy", "legacy"), home, {"DDB_COPC_BACKEND": "untwine"})
        p = subprocess.run([os.path.join(common.BASELINE_DIR, "usr", "bin", "ddb"), "copc", out_dir, item["path"],
                            "--overwrite"], env=env, capture_output=True, text=True)
        if p.returncode != 0 or not os.path.isfile(out):
            raise RuntimeError("COPC preparation of %s failed: %s" % (item["id"], p.stderr[-2000:]))
        self.remove_dir(home)
        return out

    def plan(self):
        a, prof, suite = self.a, self.profile, self.suite
        only_items = csv_list(a.items)
        items = [i for i in suite["items"] if not only_items or i in only_items]
        tile_cases, build_cases = [], []
        scen_filter = csv_list(a.scenarios)
        for iid in items:
            item = self.items.get(iid)
            if item is None:
                common.log("item %s not in the manifest: skipped" % iid)
                continue
            flags = suite["item_flags"].get(iid, {})
            fp, minz, maxz = common.dataset_geometry(item)
            if a.only in (None, "tile") and flags.get("tile", True):
                if fp is None or (item.get("probe") or {}).get("footprint_3857", {}).get("error"):
                    common.log("item %s has no usable EPSG:3857 footprint (outside the world, e.g. E1): no tile cases" % iid)
                else:
                    for scen in suite["tile_scenarios"]:
                        if scen_filter and scen not in scen_filter:
                            continue
                        sp = prof["tile"].get(scen, {})
                        size = SCENARIO_SIZE[scen]
                        if suite["matrix"].get("tile_sizes") and size not in suite["matrix"]["tile_sizes"]:
                            continue
                        for fmt in narrow(sp.get("formats"), suite["matrix"].get("formats"), csv_list(a.formats)):
                            for th in narrow(sp.get("threads"), suite["matrix"].get("threads"),
                                             csv_list(a.threads, int)):
                                for cache in narrow(sp.get("cache"), suite["matrix"].get("cache"),
                                                    csv_list(a.cache)):
                                    tile_cases.append({"kind": "tile", "dataset": iid, "scenario": scen,
                                                       "size": size, "format": fmt, "threads": int(th),
                                                       "cache": cache, "zoom": [minz, maxz]})
            if a.only in (None, "build") and flags.get("build", True):
                bscens = [s for s in suite["build_scenarios"]
                          if (s == "cog") == (item.get("kind") == "raster")]
                for scen in bscens:
                    if scen_filter and scen not in scen_filter:
                        continue
                    for th in narrow(prof["build"].get("threads"), suite["matrix"].get("threads"),
                                     csv_list(a.threads, int)):
                        build_cases.append({"kind": "build", "dataset": iid, "scenario": scen, "threads": int(th)})
        for c in tile_cases:
            c["id"] = "tile/%s/%s/%d/%s/%dT/%s" % (c["dataset"], c["scenario"], c["size"], c["format"], c["threads"],
                                                   c["cache"])
        for c in build_cases:
            c["id"] = "build/%s/%s/%dT" % (c["dataset"], c["scenario"], c["threads"])
        return tile_cases, build_cases

    def request_file(self, item, scen, path, minz, maxz):
        fn = os.path.join(self.req_dir, "%s-%s.jsonl" % (item["id"], scen))
        if os.path.isfile(fn):
            return fn
        fp, _, _ = common.dataset_geometry(item)
        p = self.profile
        reqs = reqgen.generate(scen, path, fp, minz, maxz, "png", int(p.get("seed", 1)), int(p.get("sweep_cap", 256)),
                               int(p.get("pan_steps", 200)), int(p.get("pan_min_requests", 2000)))
        reqgen.write_jsonl(reqs, fn)
        return fn

    # ---- tile cases
    def tile_invocation(self, case, backend, reqfile, home, out_json, rounds, warmups, limit=None, dump=None,
                        prepare=False):
        pin = common.pin_for_threads(self.pins, case["threads"])
        cmd = [self.driver, "tile", "--backend", backend.kind, "--lib", backend.lib, "--requests", reqfile,
               "--threads", str(case["threads"]), "--pin", common.cpulist(pin), "--warmup-rounds", str(warmups),
               "--rounds", str(rounds), "--format", case["format"], "--size", str(case["size"]), "--hashes",
               "--out", out_json]
        if limit:
            cmd += ["--limit", str(limit)]
        if dump:
            cmd += ["--dump-tiles", dump]
        perf_file = None
        if self.a.perf_stat and not limit:
            perf_file = out_json + ".perf.csv"
            cmd = perf_wrap(cmd, perf_file)
        if prepare:
            # the COG build of a raw input (first tile) runs on the 8 CCD0 cores whatever the case's threads
            i = cmd.index("--threads")
            cmd[i + 1] = "1"
            j = cmd.index("--pin")
            del cmd[j:j + 2]
            cmd = ["taskset", "-c", common.cpulist(self.pins["8T"])] + cmd
        elif self.gate_cpus:  # library threads that the driver does not pin (and helpers) stay on the gate CPUs
            cmd = ["taskset", "-c", common.cpulist(self.gate_cpus)] + cmd
        env = self.env_for(backend, home)
        hw = gates.HostWindow(self.gate_cpus) if self.gate_cpus else None
        t0 = time.monotonic()
        p = subprocess.run(cmd, env=env, capture_output=True, text=True)
        wall = time.monotonic() - t0
        host = hw.stop() if hw else None
        if p.returncode != 0 or not os.path.isfile(out_json):
            raise RuntimeError("tile run failed (%d): %s" % (p.returncode, p.stderr.strip()[-2000:]))
        res = common.read_json(out_json)
        res["_wall_s"] = wall
        res["_host"] = host
        res["_perf"] = parse_perf(perf_file) if perf_file else None
        return res, pin

    @staticmethod
    def tile_metrics(res):
        meas = [r for r in res["round_results"] if not r.get("warmup")]
        m = {}
        for k in ("p50", "p90", "p95", "p99", "p999", "max", "mean"):
            vals = [r["latency_ms"].get(k) for r in meas if r["latency_ms"].get(k) is not None]
            m[k + "_ms"] = stats.median(vals) if vals else None
        for k in ("p50", "p99"):  # successful requests only (libddb fails out-of-bounds COPC tiles)
            vals = [(r.get("latency_ok_ms") or {}).get(k) for r in meas]
            vals = [v for v in vals if v is not None]
            m[k + "_ok_ms"] = stats.median(vals) if vals else None
        m["tiles_per_s"] = stats.median([r["tiles_per_s"] for r in meas])
        m["cpu_s_per_round"] = stats.median([r["cpu"]["user_s"] + r["cpu"]["sys_s"] for r in meas])
        m["rss_peak_kb"] = max(r["rss_kb"]["peak"] for r in meas)
        m["hwm_kb"] = res.get("rss_kb", {}).get("hwm_end")
        m["failures"] = sum(r["failures"] for r in meas)
        m["bytes"] = meas[0]["bytes"] if meas else None
        m["majflt"] = sum(r["rusage"]["majflt"] for r in meas)
        m["init_ms"] = res.get("init_ms")
        if res.get("_perf"):
            m.update(res["_perf"])
        return m

    def run_tile_case(self, case):
        item = self.items[case["dataset"]]
        path, needs_prepare = self.tile_input(item)
        minz, maxz = case["zoom"]
        reqfile = self.request_file(item, case["scenario"], path, minz, maxz)
        case["params"] = {"size": case["size"], "format": case["format"], "threads": case["threads"],
                          "cache": case["cache"], "requests": os.path.relpath(reqfile, self.results_dir),
                          "n_requests": sum(1 for _ in open(reqfile)), "input": path,
                          "pin": common.pin_for_threads(self.pins, case["threads"])}
        cold = case["cache"] == "cold"
        rounds = 1 if cold else (self.a.rounds or int(self.profile.get("rounds", 3)))
        warmups = 0 if cold else (self.a.warmup_rounds if self.a.warmup_rounds is not None
                                  else int(self.profile.get("warmup_rounds", 1)))
        case["params"].update({"rounds": rounds, "warmup_rounds": warmups})
        raw = os.path.join(self.raw_dir, slug(case["id"]))
        os.makedirs(raw, exist_ok=True)
        homes = {b.name: self.fresh_dir(os.path.join(self.home_root, slug(case["id"]), b.name))
                 for b in self.backends}
        out = {b.name: {"reps": [], "prepare": None} for b in self.backends}
        # One request per backend in its fresh DDB_HOME before the repetitions: converts inputs
        # that are not COGs (timed as "prepare") and initialises DDB_HOME, so that every
        # repetition starts from the same state (measured: the first use of an empty DDB_HOME
        # costs ~4 G instructions even for a COG).
        for b in self.backends:
            # A raw raster is converted once per (item, backend) and run: later cases start from a copy of that
            # DDB_HOME ("seeded"), which keeps R7/R10-sized builds out of every one of the hundreds of cases.
            seed = os.path.join(self.home_root, "seed", case["dataset"], b.name)
            key = (case["dataset"], b.name)
            seeded = needs_prepare and os.path.isdir(seed)
            if seeded:
                shutil.rmtree(homes[b.name])
                shutil.copytree(seed, homes[b.name], symlinks=True)
            self.pagecache("evict", [path])
            res, _ = self.tile_invocation(case, b, reqfile, homes[b.name],
                                          os.path.join(raw, "%s-prepare.json" % b.name), 1, 0, limit=1,
                                          prepare=needs_prepare and not seeded)
            prep = {"wall_s": res["_wall_s"], "first_tile_ms": res["round_results"][0]["latency_ms"].get("max"),
                    "ok": res["round_results"][0]["ok"] == 1, "converts_input": needs_prepare, "seeded": seeded}
            if needs_prepare and not seeded:
                self.prepare_seen[key] = prep
                shutil.copytree(homes[b.name], seed, symlinks=True)
            elif seeded:
                prep["seed_prepare"] = self.prepare_seen.get(key)
            out[b.name]["prepare"] = prep
        quiet = self.quiet_gate()
        for rep in range(self.reps):
            order = self.backends if rep % 2 == 0 else list(reversed(self.backends))
            for pos, b in enumerate(order):
                out[b.name]["reps"].append(self.tile_rep(case, b, rep, pos, path, reqfile, homes[b.name], raw, cold))
        if self.a.dump_tiles:
            for b in self.backends:
                d = os.path.join(raw, "tiles-%s" % b.name)
                self.tile_invocation(case, b, reqfile, homes[b.name], os.path.join(raw, "%s-dump.json" % b.name),
                                     1, 0, dump=d)
            if len(self.backends) >= 2:
                import tilecmp
                for b in self.backends:
                    if b.name == self.base:
                        continue
                    r = tilecmp.compare_dirs(os.path.join(raw, "tiles-%s" % self.base),
                                             os.path.join(raw, "tiles-%s" % b.name),
                                             diff_dir=os.path.join(raw, "diff-%s-vs-%s" % (b.name, self.base)))
                    case.setdefault("tilecmp", {})[b.name] = tilecmp._json_safe(r["summary"])
        for b in self.backends:
            if not self.a.keep:
                self.remove_dir(homes[b.name])
            self.add_case(case, b, out[b.name]["reps"], {"quiet_gate": quiet, "prepare": out[b.name]["prepare"]})

    def tile_rep(self, case, b, rep, pos, path, reqfile, home, raw, cold):
        rounds = case["params"]["rounds"]
        warmups = case["params"]["warmup_rounds"]
        attempts = []
        for attempt in range(self.a.max_retries + 1):
            th = self.thermal_gate()
            home_files = [os.path.join(r, f) for r, _, fs in os.walk(home) for f in fs]
            pc = self.pagecache("evict" if cold else "warm", [path] + home_files)
            sw0 = gates.swap_counters()
            out_json = os.path.join(raw, "%s-rep%d-try%d.json" % (b.name, rep, attempt))
            res, pin = self.tile_invocation(case, b, reqfile, home, out_json, rounds, warmups)
            sd = gates.swap_delta(sw0, gates.swap_counters())
            valid = (sd["pswpin"] == 0 and sd["pswpout"] == 0) or self.a.swap_policy == "record"
            r = {"rep": rep, "order": pos, "attempt": attempt, "valid": valid, "thermal": th, "pagecache": pc,
                 "swap_delta": sd, "metrics": self.tile_metrics(res), "set_hash": res.get("set_hash"),
                 "host": res.get("_host"),
                 "hash_mismatches": res.get("hash_mismatches"), "errors": res["round_results"][-1].get("errors"),
                 "raw": os.path.relpath(out_json, self.results_dir)}
            if not valid:
                r["invalid_reason"] = "swap activity %s" % sd
                common.log("rep %d of %s/%s invalid (swap): rerun" % (rep, case["id"], b.name))
            attempts.append(r)
            if valid:
                break
        final = attempts[-1]
        final["attempts"] = len(attempts)
        m = final["metrics"]
        if m.get("instructions") and case["params"].get("n_requests"):
            # whole invocation (init and warmup rounds included) over all the requests it served
            m["instr_per_tile"] = m["instructions"] / (case["params"]["n_requests"] * (rounds + warmups))
        common.log("%-48s %-10s rep %d  p50 %.2f ms  p99 %.2f ms  %.0f tiles/s  fail %d" % (
            case["id"], b.name, rep, m.get("p50_ms") or -1, m.get("p99_ms") or -1, m.get("tiles_per_s") or 0,
            m.get("failures") or 0))
        return final

    # ---- build cases
    def build_command(self, b, scen, inp, out_dir):
        if b.kind == "legacy":
            if scen == "cog":
                return [b.cli, "cog", os.path.join(out_dir, "cog.tif"), inp, "--overwrite"], \
                    os.path.join(out_dir, "cog.tif"), {}
            backend = "untwine" if scen == "copc-untwine" else "pdal"
            return [b.cli, "copc", os.path.join(out_dir, "copc"), inp, "--overwrite"], \
                os.path.join(out_dir, "copc", "cloud.copc.laz"), {"DDB_COPC_BACKEND": backend}
        if scen == "copc-pdal":
            return None, None, None  # ddbt has one COPC builder: compared in copc-untwine
        if not b.cli:
            return None, None, None
        name = "cog.tif" if scen == "cog" else "cloud.copc.laz"
        return [b.cli, "cog" if scen == "cog" else "copc", "build", inp, os.path.join(out_dir, name)], \
            os.path.join(out_dir, name), {}

    def disk_guard(self, inp_size, kind):
        est = inp_size * (3 if kind == "raster" else 2)
        os.makedirs(common.OUT_ROOT, exist_ok=True)
        free = shutil.disk_usage(common.OUT_ROOT).free
        need = est + 20 * 1024 ** 3
        if free < need:
            raise SystemExit("disk guard: %.1f GB free under %s, need %.1f GB (estimate + 20 GB)"
                             % (free / 1e9, common.OUT_ROOT, need / 1e9))
        return {"free_gb": round(free / 1e9, 1), "estimate_gb": round(est / 1e9, 2)}

    def run_build_case(self, case):
        item = self.items[case["dataset"]]
        inp = item["path"]
        cpus = common.pin_for_threads(self.pins, case["threads"])
        case["params"] = {"threads": case["threads"], "cpus": cpus, "input": inp}
        raw = os.path.join(self.raw_dir, slug(case["id"]))
        os.makedirs(raw, exist_ok=True)
        active = []
        for b in self.backends:
            cmd, _, _ = self.build_command(b, case["scenario"], inp, "/x")
            if cmd is None:
                common.log("%s: backend %s has no command for %s (skipped)" % (case["id"], b.name, case["scenario"]))
            else:
                active.append(b)
        quiet = self.quiet_gate()
        out = {b.name: [] for b in active}
        for rep in range(self.a.reps or int(self.profile["build"].get("reps", 3))):
            order = active if rep % 2 == 0 else list(reversed(active))
            for pos, b in enumerate(order):
                out[b.name].append(self.build_rep(case, b, rep, pos, item, cpus, raw))
        for b in active:
            self.add_case(case, b, out[b.name], {"quiet_gate": quiet})

    def build_rep(self, case, b, rep, pos, item, cpus, raw):
        attempts = []
        for attempt in range(self.a.max_retries + 1):
            dg = self.disk_guard(item.get("size") or os.path.getsize(item["path"]), item.get("kind"))
            out_dir = self.fresh_dir(os.path.join(common.OUT_ROOT, b.name, self.run_id, slug(case["id"]),
                                                  "rep%d" % rep))
            home = self.fresh_dir(os.path.join(self.home_root, slug(case["id"]), b.name, "rep%d" % rep))
            cmd, out_file, extra = self.build_command(b, case["scenario"], item["path"], out_dir)
            extra = dict(extra, DDB_BUILD_THREADS=str(case["threads"]))
            env = self.env_for(b, home, extra)
            th = self.thermal_gate()
            pc = self.pagecache("evict", [item["path"]])
            run_json = os.path.join(raw, "%s-rep%d-try%d.json" % (b.name, rep, attempt))
            perf_file = None
            inner = cmd
            if self.a.perf_stat:
                perf_file = run_json + ".perf.csv"
                inner = perf_wrap(cmd, perf_file)
            wrapper = [self.driver, "run", "--cpus", common.cpulist(cpus),
                       "--stdout", run_json + ".stdout", "--stderr", run_json + ".stderr", "--out", run_json, "--"]
            hw = gates.HostWindow(self.gate_cpus) if self.gate_cpus else None
            subprocess.run(wrapper + inner, env=env)
            host = hw.stop() if hw else None
            res = common.read_json(run_json)
            vm = res.get("vmstat_delta", {})
            valid = res.get("exit_code") == 0 and (self.a.swap_policy == "record" or (
                vm.get("pswpin", 0) == 0 and vm.get("pswpout", 0) == 0))
            m = {"wall_s": res["wall_s"], "user_s": res["user_s"], "sys_s": res["sys_s"],
                 "cpu_s": res["user_s"] + res["sys_s"], "tree_rss_peak_kb": res["tree"]["rss_peak_kb"],
                 "tree_pss_peak_kb": res["tree"]["pss_peak_kb"], "maxrss_kb": res["rusage"]["maxrss_kb"],
                 "read_bytes": res["io"]["read_bytes"], "write_bytes": res["io"]["write_bytes"],
                 "majflt": res["rusage"]["majflt"], "exit_code": res["exit_code"],
                 "output_bytes": os.path.getsize(out_file) if out_file and os.path.isfile(out_file) else None}
            if perf_file:
                m.update(parse_perf(perf_file))
            r = {"rep": rep, "order": pos, "attempt": attempt, "valid": valid, "thermal": th, "pagecache": pc,
                 "disk": dg, "host": host, "swap_delta": {k: vm.get(k) for k in ("pswpin", "pswpout")}, "metrics": m,
                 "raw": os.path.relpath(run_json, self.results_dir)}
            if res.get("exit_code") != 0:
                r["invalid_reason"] = "exit code %s" % res.get("exit_code")
                try:
                    with open(run_json + ".stderr") as f:
                        r["stderr_tail"] = f.read()[-1500:]
                except OSError:
                    pass
            elif not valid:
                r["invalid_reason"] = "swap activity"
            if res.get("exit_code") == 0 and out_file and (rep == 0 or self.a.validate_all):
                r["validation"] = self.validate_output(case["scenario"], out_file, item["path"], rep == 0)
            if not self.a.keep:
                self.remove_dir(out_dir)
            self.remove_dir(home)
            attempts.append(r)
            common.log("%-48s %-10s rep %d  wall %.2f s  cpu %.2f s  rss %.0f MB  %s" % (
                case["id"], b.name, rep, m["wall_s"], m["cpu_s"], m["tree_rss_peak_kb"] / 1024,
                "ok" if valid else r.get("invalid_reason")))
            if valid or res.get("exit_code") != 0:
                break  # a failing command is not retried
        attempts[-1]["attempts"] = len(attempts)
        return attempts[-1]

    def validate_output(self, scen, out_file, inp, full):
        try:
            if scen == "cog":
                import validate_cog
                r = validate_cog.validate(out_file, None, full_check=full)
                return {"ok": r["ok"], "validator": r["validator"].get("valid"), "contract": r.get("contract")}
            import validate_copc
            r = validate_copc.validate(out_file, inp, validate_copc.find_pdal(self.a.pdal))
            return {"ok": r["ok"], "diffs": r.get("diffs"), "warnings": r.get("warnings"),
                    "summary": {k: v for k, v in (r["target"].get("summary") or {}).items() if k != "srs_wkt"}}
        except Exception as e:  # noqa: BLE001 - recorded, does not stop the run
            return {"ok": False, "error": str(e)}

    # ---- bookkeeping
    def add_case(self, case, b, reps, extra):
        for r in reps:
            if r.get("host"):
                self.iso_samples.append({"where": "invocation", "other": r["host"]["other_cpus_util_pct"],
                                         "gate": r["host"]["gate_cpus_util_pct"], "load1": r["host"]["loadavg"][0]})
        metrics = sorted({k for r in reps for k, v in r["metrics"].items() if isinstance(v, (int, float))})
        valid = [r for r in reps if r["valid"]]
        summary = {k: stats.summarize([r["metrics"].get(k) for r in valid]) for k in metrics}
        summary["valid_reps"] = len(valid)
        summary["reps"] = len(reps)
        hashes = sorted({r.get("set_hash") for r in reps if r.get("set_hash")})
        if hashes:
            summary["set_hashes"] = hashes
        entry = {"id": case["id"], "kind": case["kind"], "backend": b.name, "dataset": case["dataset"],
                 "scenario": case["scenario"], "params": case.get("params", {}), "reps": reps, "summary": summary}
        entry.update({k: v for k, v in extra.items() if v is not None})
        if case.get("tilecmp", {}).get(b.name):
            entry["tilecmp_vs_base"] = case["tilecmp"][b.name]
        self.cases.append(entry)

    def comparisons(self, noise):
        out = []
        by_id = {}
        for c in self.cases:
            by_id.setdefault(c["id"], {})[c["backend"]] = c
        for cid, per in by_id.items():
            if self.base not in per:
                continue
            base = per[self.base]
            for name, head in per.items():
                if name == self.base:
                    continue
                metrics = TILE_METRICS if base["kind"] == "tile" else BUILD_METRICS
                if self.a.perf_stat:
                    metrics = metrics + ["instructions"]
                rows = []
                for m in metrics:
                    bv = [r["metrics"].get(m) for r in base["reps"] if r["valid"]]
                    hv = [r["metrics"].get(m) for r in head["reps"] if r["valid"]]
                    bv = [v for v in bv if v is not None]
                    hv = [v for v in hv if v is not None]
                    if bv and hv:
                        rows.append(stats.compare(bv, hv, m, noise))
                cmp_ = {"id": cid, "kind": base["kind"], "dataset": base["dataset"], "scenario": base["scenario"],
                        "base": self.base, "head": name, "metrics": rows}
                if base["kind"] == "tile":
                    cmp_["same_tiles"] = (base["summary"].get("set_hashes") == head["summary"].get("set_hashes")
                                          and bool(base["summary"].get("set_hashes")))
                    cmp_["failures"] = {"base": base["summary"].get("failures", {}).get("median"),
                                        "head": head["summary"].get("failures", {}).get("median")}
                out.append(cmp_)
        return out

    def isolation(self, cases):
        """run.json "isolation": the ccd0-isolated record (gate CPUs, load of the other CCD sampled at every
        gate and around every invocation, Tctl samples)."""
        if not self.gate_cpus:
            return None
        gate = set(self.gate_cpus)
        outside = sorted({c for case in cases for c in (case.get("params", {}).get("pin") or
                                                           case.get("params", {}).get("cpus") or [])
                          if c not in gate})

        def agg(where):
            v = [x["other"]["mean"] for x in self.iso_samples if x["where"] == where and x["other"]["mean"] is not None]
            return {"samples": len(v), "mean_pct": round(sum(v) / len(v), 2) if v else None,
                    "min_pct": round(min(v), 2) if v else None, "max_pct": round(max(v), 2) if v else None}

        g = [x["gate"]["max"] for x in self.iso_samples if x["where"] == "quiet-gate" and x["gate"]["max"] is not None]
        t = [x for x in self.temps if x is not None]
        return {"mode": "ccd0-isolated" if not outside else "gate-cpus-partial",
                "gate_cpus": common.cpulist(self.gate_cpus), "pins_outside_gate_cpus": outside,
                "other_cpus_util_during_invocations": agg("invocation"),
                "other_cpus_util_at_quiet_gates": agg("quiet-gate"),
                "gate_cpus_max_cpu_util_at_quiet_gates": {"mean_pct": round(sum(g) / len(g), 2) if g else None,
                                                          "max_pct": round(max(g), 2) if g else None},
                "loadavg_start": self.load_start, "loadavg_end": gates.loadavg(),
                "tctl_c": {"samples": len(t), "min": min(t) if t else None, "max": max(t) if t else None,
                           "mean": round(sum(t) / len(t), 1) if t else None},
                "thermal_policy": self.a.gate_policy, "thermal_max_c": self.a.thermal_max}

    def write_results(self, status, guard):
        noise = stats.load_noise()
        run = {
            "schema": "ddbt-bench/1",
            "run_id": self.run_id,
            "status": status,
            "created": common.utc_iso(),
            "suite": {"name": self.suite["name"], "file": self.suite["file"], "profile": self.profile,
                      "manifest": self.manifest_path},
            "machine": self.sysinfo,
            "build": {"ddb_tilers_sha": common.git_sha(), "driver": self.driver,
                      "backends": {b.name: b.describe() for b in self.backends}, "base": self.base},
            "baseline": {"dir": common.BASELINE_DIR, "env_file": self.env_file,
                         "manifest": self.sysinfo.get("baseline", {}).get("manifest"),
                         "manifest_sha256": self.sysinfo.get("baseline", {}).get("manifest_sha256")},
            "settings": {k: v for k, v in vars(self.a).items() if not k.startswith("_")},
            "pin_sets": self.pins,
            "guards": guard,
            "isolation": self.isolation(self.cases),
            "noise": noise,
            "cases": self.cases,
            "comparisons": self.comparisons(noise),
            "events": self.events,
        }
        common.write_json(os.path.join(self.results_dir, "run.json"), run)
        import report
        md = report.render(run)
        with open(os.path.join(self.results_dir, "report.md"), "w") as f:
            f.write(md)
        if not self.a.no_summary_copy:
            dst = os.path.join(common.REPO, "bench", "results", self.run_id)
            os.makedirs(dst, exist_ok=True)
            summary = dict(run)
            summary["cases"] = [{k: v for k, v in c.items() if k != "reps"} for c in self.cases]
            summary["machine"] = {k: self.sysinfo.get(k) for k in ("hostname", "kernel", "os", "power", "memory")}
            summary["machine"]["cpu"] = {k: self.sysinfo.get("cpu", {}).get(k)
                                         for k in ("model", "isa_level", "logical_cpus", "l3_groups")}
            common.write_json(os.path.join(dst, "summary.json"), summary)
            with open(os.path.join(dst, "report.md"), "w") as f:
                f.write(md)
        return run

    def run(self):
        tile_cases, build_cases = self.plan()
        n_inv = len(tile_cases) * self.reps * len(self.backends) + len(build_cases) * len(self.backends) * (
            self.a.reps or int(self.profile["build"].get("reps", 3)))
        common.log("run %s: %d tile cases, %d build cases, backends %s, ~%d invocations" % (
            self.run_id, len(tile_cases), len(build_cases), [b.name for b in self.backends], n_inv))
        if self.a.dry_run:
            for c in tile_cases + build_cases:
                print(c["id"])
            return 0
        os.makedirs(self.results_dir, exist_ok=True)
        inputs = [self.items[c["dataset"]]["path"] for c in tile_cases + build_cases]  # work copies may change
        snap0 = snapshot_dirs(sorted(set(inputs)))
        guard = {"readonly": {"checked_paths": len(snap0), "ok": True}}
        status = "ok"
        try:
            for case in tile_cases + build_cases:
                try:
                    if case["kind"] == "tile":
                        self.run_tile_case(case)
                    else:
                        self.run_build_case(case)
                except RuntimeError as e:
                    common.log("case %s failed: %s" % (case["id"], e))
                    self.events.append({"case": case["id"], "error": str(e)})
                diff = snapshot_diff(snap0, snapshot_dirs(sorted(set(inputs))))
                if diff:
                    guard["readonly"] = {"ok": False, "changed": diff[:50], "after_case": case["id"]}
                    status = "readonly-violation"
                    common.log("READ-ONLY GUARD: inputs changed after %s: %s" % (case["id"], diff[:5]))
                    break
        except KeyboardInterrupt:
            status = "interrupted"
        finally:
            if not self.a.keep:
                if os.path.isdir(self.home_root):
                    shutil.rmtree(self.home_root, ignore_errors=True)
                for b in self.backends:
                    self.remove_dir(os.path.join(common.OUT_ROOT, b.name, self.run_id))
            self.write_results(status, guard)
        common.log("results: %s" % self.results_dir)
        return {"ok": 0, "readonly-violation": 3, "interrupted": 130}[status]


def build_argparser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--suite", required=True, help="suite name (bench/suites/<name>.toml) or path")
    ap.add_argument("--manifest", help="corpus manifest (default: the suite's corpus_manifest)")
    ap.add_argument("--backend", action="append",
                    help="name=kind:lib[:cli[:env]], or legacy / ddbt with the default paths (repeatable; default legacy)")
    ap.add_argument("--base", help="backend name used as the base of comparisons (default: the first)")
    ap.add_argument("--aa", action="store_true", help="A/A noise run: the first backend twice")
    ap.add_argument("--driver", help="ddbt-bench executable")
    ap.add_argument("--only", choices=("tile", "build"))
    ap.add_argument("--items", help="comma-separated subset of the suite items")
    ap.add_argument("--scenarios", help="comma-separated subset: sweep,pan,retina,edge,cog,copc-untwine,copc-pdal")
    ap.add_argument("--threads", help="override the thread counts, e.g. 1,8")
    ap.add_argument("--formats", help="override the tile formats")
    ap.add_argument("--cache", help="override the cache states (warm,cold)")
    ap.add_argument("--profile", help="profiles.toml section to use instead of the suite name (e.g. baseline, aa)")
    ap.add_argument("--gate-cpus", help="CPU list (e.g. 0-7,16-23) for a run pinned to one CCD while the other CCD is "
                    "busy: the quiet gate looks at the per-CPU utilisation of these CPUs only (/proc/stat), the "
                    "load of the other CPUs is recorded, and run.json gets isolation.mode = ccd0-isolated")
    ap.add_argument("--pin-set", action="append", metavar="NT=SET",
                    help="pin the N-thread cases on another named set, e.g. 16T=16Tsmt (CCD0's 16 logical CPUs, so "
                         "16 threads stay inside --gate-cpus 0-7,16-23); the sets are those of common.pin_sets")
    ap.add_argument("--tile-path", action="append", metavar="ID=PATH",
                    help="tile a work copy of an item instead of the corpus file (libddb writes a stats.json "
                         "sidecar next to a COG on its first non-Byte tile, which must never happen under "
                         "/mnt/data/datasets); the original stays in the read-only guard")
    ap.add_argument("--reps", type=int)
    ap.add_argument("--rounds", type=int)
    ap.add_argument("--warmup-rounds", type=int)
    ap.add_argument("--max-retries", type=int, default=2, help="reruns of a repetition invalidated by swapping")
    ap.add_argument("--perf-stat", action="store_true",
                    help="wrap invocations with perf stat (%s)" % PERF_EVENTS)
    ap.add_argument("--dump-tiles", action="store_true", help="dump the tiles of each backend and run tilecmp")
    ap.add_argument("--validate-all", action="store_true", help="validate every build output (default: rep 0)")
    ap.add_argument("--pdal", help="pdal executable for validate_copc.py")
    ap.add_argument("--ddbt-env", action="append", help="K=V added to the environment of ddbt backends")
    ap.add_argument("--keep", action="store_true", help="keep build outputs and DDB_HOME folders")
    ap.add_argument("--gate-policy", choices=("strict", "warn", "off"), default="warn",
                    help="strict: abort when a gate times out; warn: record it and go on; off: no gates")
    ap.add_argument("--swap-policy", choices=("invalidate", "record"), default="invalidate",
                    help="invalidate: a global pswpin/pswpout delta during a repetition marks it invalid (rerun); "
                         "record: only recorded (swap_delta), for a shared host where other lanes' memory pressure "
                         "swaps constantly while this run's own RSS is far below the free memory")
    ap.add_argument("--quiet-timeout", type=float, default=300.0)
    ap.add_argument("--load-max", type=float, default=1.0)
    ap.add_argument("--proc-max-pct", type=float, default=10.0)
    ap.add_argument("--thermal-max", type=float, default=55.0)
    ap.add_argument("--thermal-timeout", type=float, default=120.0)
    ap.add_argument("--run-id")
    ap.add_argument("--results-root")
    ap.add_argument("--no-summary-copy", action="store_true", help="do not write bench/results/<run-id>/")
    ap.add_argument("--dry-run", action="store_true", help="print the case list and exit")
    return ap


def _sigterm(signum, frame):
    raise KeyboardInterrupt  # same path as Ctrl-C: results written, work folders removed


def main(argv=None):
    a = build_argparser().parse_args(argv)
    signal.signal(signal.SIGTERM, _sigterm)
    return Runner(a).run()


if __name__ == "__main__":
    sys.exit(main())
