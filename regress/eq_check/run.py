#!/usr/bin/env python3
"""Regression runner: run main.py + sby equivalence (PO-only) for all benchmarks."""

import subprocess, sys, shutil, tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
OSS_BIN = ROOT / "oss-cad-suite" / "bin"
YOSYS = str(OSS_BIN / "yosys")
SBY = str(OSS_BIN / "sby")
SV2V = str(OSS_BIN / "sv2v")
MAIN_PY = str(ROOT / "src" / "main.py")

BENCHMARKS = {
    "adder":    {"file": "bench/adder/adder.v",          "top": "adder_32bit"},
    "b17":      {"file": "bench/b17/b17.v",              "top": "b17"},
    "b19":      {"file": "bench/b19/b19.v",              "top": "b19"},
    "b21":      {"file": "bench/b21/b21.v",              "top": "b21"},
    "b22":      {"file": "bench/b22/b22.v",              "top": "b22"},
    "picorv32": {"file": "bench/cascade_picorv32/pico.v", "top": "picorv32"},
    "rocket":   {"file": "bench/cascade_rocket/rocket.v", "top": "RocketTile"},
}

# Benchmarks that need sv2v pre-processing (yosys can't parse raw source)
NEED_SV2V = {"b17", "b19", "b21", "b22", "picorv32", "rocket"}


def run_cmd(cmd, cwd=None, timeout=600, env=None):
    proc = subprocess.run(cmd, capture_output=True, text=True, cwd=cwd, timeout=timeout, env=env)
    return proc.returncode, proc.stdout, proc.stderr


def check_one(name, info, work_dir):
    """Step 1: run main.py. Step 2: sby equivalence check."""
    test_dir = ROOT / "regress" / "eq_check" / name
    test_dir.mkdir(parents=True, exist_ok=True)

    golden_src = ROOT / info["file"]
    revised = test_dir / f"f_{name}.v"
    top = info["top"]

    results = {"name": name, "mainpy": None, "equiv": None, "error": None}

    # ===== Step 1: Run main.py =====
    print(f"  [{name}] main.py ...", end=" ", flush=True)
    cmd = ["python", MAIN_PY, "--file", info["file"], "--top", top, "-o", str(revised), "--force"]
    rc, out, err = run_cmd(cmd)
    if rc != 0 or not revised.exists():
        results["mainpy"] = "FAIL"
        results["error"] = f"main.py failed (rc={rc})"
        print("FAIL")
        return results
    print("OK")

    # ===== Step 2: sby equivalence check =====
    print(f"  [{name}] sby equivalence ...", end=" ", flush=True)

    bench_work = work_dir / name
    bench_work.mkdir(parents=True, exist_ok=True)

    # Prepare golden source (sv2v if needed)
    golden = bench_work / f"{name}_golden.v"
    if name in NEED_SV2V:
        rc, stdout, stderr = run_cmd([SV2V, str(golden_src)])
        if rc != 0:
            results["equiv"] = "ERROR"
            results["error"] = f"sv2v failed: {stderr[-200:]}"
            print("ERROR")
            return results
        golden.write_text(stdout)
    else:
        shutil.copy2(golden_src, golden)

    # Copy revised file
    shutil.copy2(revised, bench_work / f"f_{name}.v")

    # Write config.sby (PO-only miter)
    sby_script = f"""\
[options]
mode prove
depth 10

[engines]
smtbmc

[script]
read -formal {name}_golden.v
prep -top {top}
design -stash golden

read -formal f_{name}.v
prep -top {top}
design -stash revised

design -load golden
design -copy-from revised -as revised {top}
miter -equiv -flatten -make_outputs {top} revised miter
prep -top miter

[files]
{name}_golden.v
f_{name}.v
"""
    (bench_work / "config.sby").write_text(sby_script)

    # Run sby
    env = {"PATH": f"{OSS_BIN}:{subprocess.os.environ.get('PATH', '')}"}
    try:
        rc, stdout, stderr = run_cmd(
            [SBY, "-f", "config.sby"], cwd=bench_work, timeout=600, env=env
        )
        output = stdout + stderr
        if "DONE (PASS" in output:
            results["equiv"] = "PASS"
            print("PASS")
        elif "DONE (FAIL" in output:
            results["equiv"] = "FAIL"
            for line in output.splitlines():
                if "Assert failed" in line or "failed assertion" in line:
                    results["error"] = line.strip()
                    break
            if not results["error"]:
                results["error"] = "not equivalent"
            print("FAIL")
        else:
            results["equiv"] = "ERROR"
            err_lines = [l for l in output.splitlines() if "ERROR" in l or "Traceback" in l]
            results["error"] = "\n".join(err_lines[-5:]) if err_lines else output[-300:]
            print("ERROR")
    except subprocess.TimeoutExpired:
        results["equiv"] = "TIMEOUT"
        results["error"] = "sby timed out after 600s"
        print("TIMEOUT")

    return results


def main():
    work_dir = Path(tempfile.mkdtemp(prefix="sby_regress_"))

    passed = failed = errored = 0
    print("=" * 60)
    print("VeriFlatten Regression Tests")
    print("=" * 60)
    print()

    for name, info in BENCHMARKS.items():
        print(f"[{name}] (top={info['top']})")
        r = check_one(name, info, work_dir)
        if r["equiv"] == "PASS":
            passed += 1
        elif r["equiv"] == "FAIL":
            failed += 1
        else:
            errored += 1
        if r["error"]:
            print(f"       Error: {r['error'][:200]}")
        print()

    print("=" * 60)
    print(f"Results: {passed} passed, {failed} failed, {errored} errored")
    print("=" * 60)
    return 0 if failed == 0 and errored == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
