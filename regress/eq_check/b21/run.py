#!/usr/bin/env python3
"""b21 equivalence check using yosys."""
import subprocess, sys
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = TEST_DIR.parents[3]
YOSYS = "/home/ziyue/miniconda3/bin/yosys"
YOSYS = "/home/ziyue/miniconda3/bin/yosys"
top = "b21"
golden = TEST_DIR / "b21.v"
revised = TEST_DIR / "f_b21.v"
golden_src = PROJECT_ROOT / "bench" / "b21" / "b21.v"

def main():
    if not golden.exists():
        subprocess.run(["cp", str(golden_src), str(golden)], check=True)
    if not revised.exists():
        print(f"Generating f_b21.v...")
        rc = subprocess.run(
            ["python", "src/main.py", "--file", f"bench/b21/b21.v",
             "--top", top, "-o", str(revised), "--force"],
            capture_output=True, text=True, cwd=PROJECT_ROOT
        ).returncode
        if rc != 0:
            print("FAIL: main.py error"); return 1
        print("  OK")
    print("Running yosys equivalence...")
    ys = f"""
read -formal {golden}
prep -flatten -top {top}
design -stash golden_A
read -formal {revised}
prep -flatten -top {top}
design -stash revised_A
design -load golden_A
design -copy-from revised_A -as revised_A {top}
miter -equiv -flatten -make_assert -make_outputs {top} revised_A miter
prep -flatten -top miter
sat -prove-asserts miter
"""
    proc = subprocess.run([YOSYS, "-p", ys], capture_output=True, text=True, timeout=600)
    for line in proc.stdout.splitlines():
        if "SUCCESS" in line or "FAIL" in line or "SAT proof" in line:
            print(f"  {line}")
    if "SAT proof finished - no model found: SUCCESS" in proc.stdout:
        print(f"\nPASS: b21 equivalence verified OK"); return 0
    elif "SAT proof finished - model found: FAIL" in proc.stdout:
        print(f"\nFAIL: b21 NOT equivalent"); return 1
    print(f"\nFAIL: Check could not complete"); return 1

if __name__ == "__main__":
    sys.exit(main())
