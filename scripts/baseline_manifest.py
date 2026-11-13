#!/usr/bin/env python3
"""Write MANIFEST.json of the frozen baseline snapshot (and verify it with --verify).

The manifest maps every file of the extracted DroneDB .deb to sha256 and size, records the deb
checksum, the version strings the binaries report, the libddb ABI and where every shared object of
every ELF file of the snapshot resolves (inside the snapshot or the system, never anywhere else).
Run with the environment of baseline.env sourced so the resolution is the one a baseline run sees.

Usage: baseline_manifest.py --snapshot DIR --deb FILE --out MANIFEST.json [--verify]
"""
import argparse
import ctypes
import hashlib
import json
import os
import re
import subprocess
import sys
import time


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def is_elf(path):
    try:
        with open(path, "rb") as f:
            return f.read(4) == b"\x7fELF"
    except OSError:
        return False


def walk(root):
    for d, dirs, files in os.walk(root):
        dirs.sort()
        for n in sorted(files):
            yield os.path.join(d, n)


def file_table(snap):
    table = {}
    for p in walk(snap):
        rel = os.path.relpath(p, snap)
        if os.path.islink(p):
            table[rel] = {"symlink": os.readlink(p)}
        else:
            table[rel] = {"sha256": sha256(p), "size": os.path.getsize(p)}
    return table


def resolve_libs(snap):
    """ldd every ELF of the snapshot; return (per-file resolution, system libs, offenders)."""
    system, offenders, per_file = set(), [], {}
    for p in walk(snap):
        if os.path.islink(p) or not is_elf(p):
            continue
        out = subprocess.run(["ldd", p], capture_output=True, text=True).stdout
        libs = {}
        for line in out.splitlines():
            m = re.match(r"\s*(\S+) => (\S+)", line)
            if not m:
                continue
            name, path = m.groups()
            if path == "not":
                libs[name] = "NOT FOUND"
                offenders.append((os.path.relpath(p, snap), name, "not found"))
                continue
            libs[name] = path
            if path.startswith(snap + os.sep):
                continue
            if path.startswith("/mnt/data/repos") or path.startswith("/usr/local"):
                offenders.append((os.path.relpath(p, snap), name, path))
            else:
                system.add(os.path.basename(path))
        per_file[os.path.relpath(p, snap)] = libs
    return per_file, sorted(system), offenders


def tool_version(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    return (r.stdout + r.stderr).strip().splitlines()[0] if (r.stdout + r.stderr).strip() else ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--snapshot", required=True)
    ap.add_argument("--deb", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--verify", action="store_true", help="verify the files against an existing manifest")
    a = ap.parse_args()
    snap = os.path.realpath(a.snapshot)

    if a.verify:
        m = json.load(open(a.out))
        bad = 0
        cur = file_table(snap)
        for rel, e in m["files"].items():
            if cur.get(rel) != e:
                print("MISMATCH", rel)
                bad += 1
        for rel in cur:
            if rel not in m["files"]:
                print("EXTRA", rel)
                bad += 1
        print("manifest verify: %d files, %d problems" % (len(m["files"]), bad))
        sys.exit(1 if bad else 0)

    per_file, system, offenders = resolve_libs(snap)
    if offenders:
        for o in offenders:
            print("UNRESOLVED/OUTSIDE:", *o, file=sys.stderr)
        sys.exit("snapshot resolves libraries outside itself and the system: see above")

    lib = ctypes.CDLL(os.path.join(snap, "usr/lib/libddb.so"))
    lib.DDBGetVersion.restype = ctypes.c_char_p
    lib.DDBGetAbiVersion.restype = ctypes.c_uint32
    abi = lib.DDBGetAbiVersion()
    ddb = os.path.join(snap, "usr/bin/ddb")
    manifest = {
        "version": 1,
        "generated": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "baseline": "DroneDB 2.4.1 official release (git cbc4bffb), libddb ABI 2.4",
        "deb": {"file": os.path.basename(a.deb), "sha256": sha256(a.deb), "size": os.path.getsize(a.deb)},
        "libddb_version": lib.DDBGetVersion().decode(),
        "libddb_abi": "0x%08x" % abi,
        "libddb_abi_text": "%d.%d" % (abi >> 16, abi & 0xFFFF),
        "ddb_version": tool_version([ddb, "--version"]),
        "gdal_util_version": tool_version([os.path.join(snap, "usr/bin/gdal-util"), "--version"]),
        "system_libs": system,
        "files": file_table(snap),
        "ldd": per_file,
    }
    with open(a.out, "w") as f:
        json.dump(manifest, f, indent=1, sort_keys=False)
        f.write("\n")
    print("wrote %s: %d files, system libs: %s" % (a.out, len(manifest["files"]), ", ".join(system)))


if __name__ == "__main__":
    main()
