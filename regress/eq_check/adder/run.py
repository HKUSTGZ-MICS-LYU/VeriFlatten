#!/usr/bin/env python3
"""
Adder equivalence check using yosys.
Verifies main.py's flattened output (f_adder.v) == original (adder.v).
"""
import subprocess, sys
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = TEST_DIR.parents[3]
YOSYS = "/home/ziyue/miniconda3/bin/yosys"

def main():
    top = "adder_32bit"
    golden = TEST_DIR / "adder.v"
    revised = TEST_DIR / "f_adder.v"
    golden_src = PROJECT_ROOT / "bench" / "adder" / "adder.v"

    if not golden.exists():
        subprocess.run(["cp", str(golden_src), str(golden)], check=True)

    if not revised.exists():
        print("Generating f_adder.v...")
        rc = subprocess.run(
            ["python", "src/main.py", "--file", "bench/adder/adder.v",
             "--top", top, "-o", str(revised), "--force"],
            capture_output=True, text=True, cwd=PROJECT_ROOT
        ).returncode
        if rc != 0:
            print("FAIL: main.py error")
            return 1
        print("  f_adder.v generated OK")

    print("Running yosys equivalence check...")
    yosys_script = f"""
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
    proc = subprocess.run(
        [YOSYS, "-p", yosys_script],
        capture_output=True, text=True, timeout=120
    )
    for line in proc.stdout.splitlines():
        if "SUCCESS" in line or "FAIL" in line or "SAT proof" in line or "prove-asserts" in line.lower():
            print(f"  {line}")
    if "SAT proof finished - no model found: SUCCESS" in proc.stdout:
        print("\nPASS: Adder equivalence verified OK")
        return 0
    elif "SAT proof finished - model found: FAIL" in proc.stdout:
        print("\nFAIL: Adder NOT equivalent")
        return 1
    else:
        print("\nFAIL: Equivalence check could not complete")
        return 1

if __name__ == "__main__":
    sys.exit(main())
