# SPDX-License-Identifier: MPL-2.0
"""Measurement gates: quiet machine, thermal headroom, no swapping."""
import glob
import os
import time


def _proc_cpu_ticks():
    """{pid: (comm, utime+stime ticks)} of every process (threads summed by the kernel)."""
    out = {}
    for d in glob.glob("/proc/[0-9]*"):
        try:
            with open(d + "/stat") as f:
                s = f.read()
        except OSError:
            continue
        rp = s.rfind(")")
        if rp < 0:
            continue
        comm = s[s.find("(") + 1:rp]
        fields = s[rp + 2:].split()
        try:
            out[int(d[6:])] = (comm, int(fields[11]) + int(fields[12]))
        except (IndexError, ValueError):
            continue
    return out


def loadavg():
    try:
        with open("/proc/loadavg") as f:
            return [float(x) for x in f.read().split()[:3]]
    except OSError:
        return [0.0, 0.0, 0.0]


def parse_cpulist(text):
    """'0-7,16-23' -> [0, 1, ..., 7, 16, ..., 23] (sorted, unique)."""
    out = set()
    for part in str(text).split(","):
        part = part.strip()
        if not part:
            continue
        lo, _, hi = part.partition("-")
        out.update(range(int(lo), int(hi or lo) + 1))
    return sorted(out)


def cpu_times():
    """{cpu: (busy_ticks, total_ticks)} from the per-CPU lines of /proc/stat (idle and iowait count as idle)."""
    out = {}
    try:
        with open("/proc/stat") as f:
            for line in f:
                if not line.startswith("cpu") or line.startswith("cpu "):
                    continue
                parts = line.split()
                v = [int(x) for x in parts[1:9]]  # user nice system idle iowait irq softirq steal
                total = sum(v)
                out[int(parts[0][3:])] = (total - v[3] - v[4], total)
    except OSError:
        pass
    return out


def cpu_util(before, after, cpus=None):
    """{cpu: busy %} between two cpu_times() snapshots (cpus: restrict to these)."""
    res = {}
    for c, (b1, t1) in after.items():
        if cpus is not None and c not in cpus:
            continue
        if c not in before:
            continue
        b0, t0 = before[c]
        res[c] = 100.0 * (b1 - b0) / (t1 - t0) if t1 > t0 else 0.0
    return res


def util_summary(util):
    """{mean, max} of a {cpu: pct} map (None when empty)."""
    if not util:
        return {"mean": None, "max": None}
    v = list(util.values())
    return {"mean": round(sum(v) / len(v), 2), "max": round(max(v), 2)}


class HostWindow:
    """Utilisation of the gate CPUs and of the other CPUs over a window (used around each invocation of a
    ccd0-isolated run: the load of the other CCD is recorded, not gated)."""

    def __init__(self, gate_cpus):
        self.gate = set(gate_cpus)
        self.t0 = cpu_times()
        self.l0 = loadavg()

    def stop(self):
        t1 = cpu_times()
        mine = cpu_util(self.t0, t1, self.gate)
        other = {c: u for c, u in cpu_util(self.t0, t1).items() if c not in self.gate}
        return {"gate_cpus_util_pct": util_summary(mine), "other_cpus_util_pct": util_summary(other),
                "loadavg": loadavg()}


def quiet_sample(window_s=2.0, proc_max_pct=10.0, exclude=(), gate_cpus=None):
    """One quiet check: load1 and the processes using more than proc_max_pct of one CPU over
    window_s seconds (this process and `exclude` pids are ignored).

    With gate_cpus (a CCD-isolated run) the check looks at those CPUs only: the CPUs of /proc/stat
    busier than proc_max_pct over the window are reported in "busy" (comm "cpuN"); load1 and the
    mean utilisation of the other CPUs are recorded ("other_cpus_util_pct") but not gated."""
    if gate_cpus:
        gate = set(gate_cpus)
        t0 = cpu_times()
        time.sleep(window_s)
        t1 = cpu_times()
        mine = cpu_util(t0, t1, gate)
        other = {c: u for c, u in cpu_util(t0, t1).items() if c not in gate}
        busy = [{"pid": 0, "comm": "cpu%d" % c, "cpu_pct": round(u, 1)}
                for c, u in sorted(mine.items(), key=lambda kv: -kv[1]) if u > proc_max_pct]
        return {"load1": loadavg()[0], "busy": busy[:10], "gate_cpus_util_pct": util_summary(mine),
                "other_cpus_util_pct": util_summary(other)}
    hz = os.sysconf("SC_CLK_TCK")
    a = _proc_cpu_ticks()
    time.sleep(window_s)
    b = _proc_cpu_ticks()
    skip = {os.getpid(), *exclude}
    busy = []
    for pid, (comm, t1) in b.items():
        if pid in skip or pid not in a:
            continue
        pct = 100.0 * (t1 - a[pid][1]) / hz / window_s
        if pct > proc_max_pct:
            busy.append({"pid": pid, "comm": comm, "cpu_pct": round(pct, 1)})
    busy.sort(key=lambda x: -x["cpu_pct"])
    return {"load1": loadavg()[0], "busy": busy[:10]}


def is_quiet(sample, load1_max=1.0, gated_load=True):
    """load1 is ignored with gated_load=False (a gate-cpus run: the machine load is not the criterion)."""
    return (sample["load1"] < load1_max or not gated_load) and not sample["busy"]


def wait_quiet(timeout_s=300.0, load1_max=1.0, proc_max_pct=10.0, window_s=2.0, gate_cpus=None):
    """Wait until quiet (or timeout). Returns (ok, last_sample, waited_s)."""
    t0 = time.monotonic()
    while True:
        s = quiet_sample(window_s, proc_max_pct, gate_cpus=gate_cpus)
        if is_quiet(s, load1_max, gated_load=not gate_cpus):
            return True, s, time.monotonic() - t0
        if time.monotonic() - t0 >= timeout_s:
            return False, s, time.monotonic() - t0
        time.sleep(3.0)


def cpu_temp():
    """k10temp Tctl (or coretemp package / first sensor) in degrees C, None when unknown."""
    for h in glob.glob("/sys/class/hwmon/hwmon*"):
        try:
            with open(h + "/name") as f:
                name = f.read().strip()
        except OSError:
            continue
        if name not in ("k10temp", "zenpower", "coretemp"):
            continue
        best = None
        for lab in sorted(glob.glob(h + "/temp*_label")):
            try:
                with open(lab) as f:
                    label = f.read().strip()
                with open(lab.replace("_label", "_input")) as f:
                    v = int(f.read()) / 1000.0
            except (OSError, ValueError):
                continue
            if label in ("Tctl", "Tdie", "Package id 0"):
                return v
            best = v if best is None else best
        if best is not None:
            return best
    return None


def wait_thermal(max_c=55.0, timeout_s=120.0, poll_s=2.0):
    """Wait until the CPU is below max_c. Returns (ok, temp, waited_s)."""
    t0 = time.monotonic()
    while True:
        t = cpu_temp()
        if t is None or t < max_c:
            return True, t, time.monotonic() - t0
        if time.monotonic() - t0 >= timeout_s:
            return False, t, time.monotonic() - t0
        time.sleep(poll_s)


def swap_counters():
    v = {}
    try:
        with open("/proc/vmstat") as f:
            for line in f:
                k, _, n = line.partition(" ")
                if k in ("pswpin", "pswpout"):
                    v[k] = int(n)
    except OSError:
        pass
    return v


def swap_delta(before, after):
    return {k: after.get(k, 0) - before.get(k, 0) for k in ("pswpin", "pswpout")}
