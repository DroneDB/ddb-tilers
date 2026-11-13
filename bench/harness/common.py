# SPDX-License-Identifier: MPL-2.0
"""Shared helpers of the ddb-tilers benchmark harness (paths, manifest, driver, topology)."""
import datetime
import json
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
WORK = os.environ.get("DDBT_WORK", "/mnt/data/ddb-tilers-work")
BASELINE_ROOT = os.path.join(WORK, "baseline")
BASELINE_DIR = os.environ.get("DDBT_BASELINE_DIR", os.path.join(BASELINE_ROOT, "ddb-2.4.1"))
BASELINE_ENV = os.path.join(BASELINE_ROOT, "baseline.env")
MANIFEST = os.path.join(WORK, "corpus", "manifest.json")
RESULTS_ROOT = os.path.join(WORK, "results")
HOME_ROOT = os.path.join(WORK, "home")
OUT_ROOT = os.path.join(WORK, "out")
SUITES_DIR = os.path.join(REPO, "bench", "suites")
GUARD_DIR = os.path.join(REPO, "bench", "guard")
NOISE_FILE = os.path.join(GUARD_DIR, "noise.json")

ORIGIN_SHIFT = 20037508.342789244  # half the EPSG:3857 world width in metres


def log(*a):
    print("[ddbt-bench]", *a, file=sys.stderr, flush=True)


def utc_stamp():
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def utc_iso():
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def git_sha(repo=REPO, short=False):
    args = ["git", "-C", repo, "rev-parse"] + (["--short=7"] if short else []) + ["HEAD"]
    try:
        return subprocess.run(args, capture_output=True, text=True, check=True).stdout.strip() or "nogit"
    except (OSError, subprocess.CalledProcessError):
        return "nogit"


# --------------------------------------------------------------------------- driver

def find_driver(explicit=None):
    """Locate the ddbt-bench executable: explicit, $DDBT_BENCH_BIN, the build trees, PATH."""
    cands = [explicit, os.environ.get("DDBT_BENCH_BIN"),
             os.path.join(REPO, "build", "linux-gcc-release", "bench", "ddbt-bench"),
             os.path.join(WORK, "build-bench", "ddbt-bench")]
    for c in cands:
        if c and os.path.isfile(c) and os.access(c, os.X_OK):
            return c
    w = shutil.which("ddbt-bench")
    if w:
        return w
    raise SystemExit("ddbt-bench not found: build it (cmake -S bench -B %s/build-bench && "
                     "cmake --build %s/build-bench) or pass --driver" % (WORK, WORK))


def driver_json(driver, args, env=None, check=True):
    """Run `ddbt-bench <args> --out -` and parse its JSON stdout."""
    p = subprocess.run([driver, args[0], "--out", "-"] + args[1:], capture_output=True, text=True, env=env)
    if check and p.returncode != 0 and not p.stdout.strip():
        raise RuntimeError("ddbt-bench %s failed (%d): %s" % (args[0], p.returncode, p.stderr.strip()))
    try:
        return json.loads(p.stdout), p.returncode
    except json.JSONDecodeError:
        raise RuntimeError("ddbt-bench %s: no JSON output: %s" % (args[0], p.stderr.strip()))


# --------------------------------------------------------------------------- baseline

def baseline_env(env_file=BASELINE_ENV, base=None):
    """Environment after sourcing the baseline env file (bash), as a dict."""
    base = dict(os.environ if base is None else base)
    if not os.path.isfile(env_file):
        return base
    p = subprocess.run(["bash", "-c", '. "$1" >/dev/null && env -0', "_", env_file],
                       capture_output=True, env=base, check=True)
    env = {}
    for item in p.stdout.split(b"\0"):
        if b"=" in item:
            k, v = item.split(b"=", 1)
            env[k.decode()] = v.decode(errors="replace")
    env["GDAL_PAM_ENABLED"] = "NO"
    return env


def baseline_lib():
    for c in (os.path.join(BASELINE_DIR, "usr", "lib", "libddb.so"),):
        if os.path.isfile(c):
            return c
    return None


def default_ddbt_lib(build_dir=None):
    roots = [build_dir] if build_dir else [os.path.join(REPO, "build", "linux-gcc-release")]
    for r in roots:
        if not r:
            continue
        for sub in ("src", "", "lib"):
            c = os.path.join(r, sub, "libddbt.so")
            if os.path.isfile(c):
                return c
    return None


def default_ddbt_cli(build_dir=None):
    r = build_dir or os.path.join(REPO, "build", "linux-gcc-release")
    for sub in ("apps/cli", "bin", ""):
        c = os.path.join(r, sub, "ddbt")
        if os.path.isfile(c) and os.access(c, os.X_OK):
            return c
    return None


# --------------------------------------------------------------------------- manifest

def load_manifest(path=MANIFEST):
    with open(path) as f:
        m = json.load(f)
    items = m.get("items", []) if isinstance(m, dict) else []
    return {it["id"]: it for it in items if isinstance(it, dict) and "id" in it}, m


def _bbox(v):
    if isinstance(v, dict):
        if "error" in v and "bbox" not in v:
            return None
        v = v.get("bbox")
    if isinstance(v, (list, tuple)) and len(v) == 4 and all(isinstance(x, (int, float)) for x in v):
        return [float(x) for x in v]
    return None


def zoom_for_pixel_size(ps, tile=256):
    """GlobalMercator::zoomForPixelSize (gdal2tiles): highest zoom whose resolution >= ps."""
    initial = 2 * ORIGIN_SHIFT / tile
    for z in range(0, 31):
        if ps > initial / (2 ** z):
            return max(0, z - 1)
    return 30


def dataset_geometry(item):
    """(footprint3857 [minx,miny,maxx,maxy], minZ, maxZ) of a manifest item, defensively.

    Accepts probe.footprint3857 / probe.footprint_3857 (list or {"bbox": [...]}) and
    probe.minZoom|maxZoom / probe.zoom.{minZ,maxZ}. Missing zooms are derived from the footprint
    (minZ: the extent fits one tile) and probe pixel sizes (maxZ), else minZ + 6.
    """
    import math
    probe = item.get("probe") or {}
    fp = None
    for k in ("footprint3857", "footprint_3857", "bbox3857"):
        fp = _bbox(probe.get(k))
        if fp:
            break
    if not fp:
        return None, None, None
    zoom = probe.get("zoom") if isinstance(probe.get("zoom"), dict) else {}
    minz = probe.get("minZoom", zoom.get("minZ"))
    maxz = probe.get("maxZoom", zoom.get("maxZ"))
    w, h = fp[2] - fp[0], fp[3] - fp[1]
    if minz is None and max(w, h) > 0:
        minz = max(0, int(math.floor(math.log2(2 * ORIGIN_SHIFT / max(w, h)))))
    if maxz is None:
        ps = zoom.get("pixel_size_3857") or probe.get("pixel_size_3857")
        maxz = zoom_for_pixel_size(ps) if ps else (minz or 0) + 6
    minz = int(minz or 0)
    maxz = int(max(maxz, minz))
    return fp, minz, maxz


# --------------------------------------------------------------------------- topology

def topology(sysinfo):
    """CPU topology from `ddbt-bench sysinfo`: list of dicts cpu/core/l3/siblings/prefcore."""
    cpus = sysinfo.get("cpu", {}).get("cpus", [])
    out = []
    for c in cpus:
        sib = [int(x) for x in str(c.get("smt_siblings", c["cpu"])).replace("-", ",").split(",") if x != ""]
        out.append({"cpu": c["cpu"], "core": c.get("core", c["cpu"]), "l3": c.get("l3", 0),
                    "siblings": sib, "prefcore": c.get("prefcore_ranking", 0)})
    return out


def pin_sets(sysinfo):
    """Named CPU sets for the thread matrix.

    1T   best-ranked core of the first L3 group (CCD0), never cpu0 (interrupts, housekeeping)
    4T   four best physical cores of CCD0 (one thread each, cpu0's core excluded)
    8T   all physical cores of CCD0 (first SMT thread of each), no SMT
    16T  physical cores of both CCDs (cross-CCD), no SMT
    16Tsmt  both SMT threads of the 8 CCD0 cores
    32T  every logical CPU
    """
    topo = topology(sysinfo)
    if not topo:
        n = os.cpu_count() or 1
        allc = list(range(n))
        return {"1T": [min(1, n - 1)], "4T": allc[:4], "8T": allc[:8], "16T": allc[:16],
                "16Tsmt": allc[:16], "32T": allc}
    primary = {}  # core key -> lowest cpu id
    for c in topo:
        key = (c["l3"], c["core"])
        primary.setdefault(key, c)
        if c["cpu"] < primary[key]["cpu"]:
            primary[key] = c
    groups = sorted({c["l3"] for c in topo})
    ccd0 = groups[0]
    phys0 = sorted([c for c in primary.values() if c["l3"] == ccd0], key=lambda c: (-c["prefcore"], c["cpu"]))
    cpu0_core = next((c["core"] for c in topo if c["cpu"] == 0), None)
    phys0_no0 = [c for c in phys0 if c["core"] != cpu0_core] or phys0
    phys_all = sorted(primary.values(), key=lambda c: (c["l3"], c["cpu"]))
    sets = {
        "1T": [phys0_no0[0]["cpu"]],
        "4T": sorted(c["cpu"] for c in phys0_no0[:4]),
        "8T": sorted(c["cpu"] for c in phys0),
        "16T": sorted(c["cpu"] for c in phys_all[:16]),
        "16Tsmt": sorted(s for c in phys0 for s in c["siblings"]),
        "32T": sorted(c["cpu"] for c in topo),
    }
    return sets


def pin_for_threads(sets, threads, name=None):
    """CPU list for a thread count: an explicit set name, else the matching default set."""
    if name:
        return sets[name]
    for k in ("%dT" % threads,):
        if k in sets and len(sets[k]) >= threads:
            return sets[k]
    allc = sets.get("32T") or list(range(os.cpu_count() or 1))
    return allc[:threads]


def cpulist(cpus):
    return ",".join(str(c) for c in cpus)


# --------------------------------------------------------------------------- misc

def write_json(path, obj):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f, indent=1, sort_keys=False)
        f.write("\n")
    os.replace(tmp, path)


def read_json(path):
    with open(path) as f:
        return json.load(f)
