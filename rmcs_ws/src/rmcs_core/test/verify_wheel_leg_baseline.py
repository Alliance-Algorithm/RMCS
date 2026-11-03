"""Compare production adapters against the saved pre-refactor executable outputs."""
import math
from pathlib import Path
import subprocess
import sys
obs, gas, traditional, execution, directory = sys.argv[1:]
root = Path(directory)
for name, program, args, source in [
    ("prone", obs, ["--prone-motion-v4-odom-v2"], "observation"),
    ("wheelbipe", obs, ["--wheelbipe-v202"], "observation"),
    ("gas", gas, ["--full-vehicle-v202"], "gas"),
    ("traditional", traditional, [], "traditional"),
    ("execution", execution, [], "execution")]:
    actual = subprocess.check_output([program, *args], input=(root/(source+".input")).read_text(), text=True)
    expected = (root/(name+".expected")).read_text()
    rows, reference = actual.splitlines(), expected.splitlines()
    assert len(rows) == len(reference), name
    for i, (row, saved) in enumerate(zip(rows, reference)):
        values, baseline = list(map(float,row.split())), list(map(float,saved.split()))
        assert len(values) == len(baseline), (name, i)
        for j, (x, y) in enumerate(zip(values, baseline)):
            assert (abs(x-y)<1e-12 if name in ("traditional","execution") else x==y) or (math.isnan(x) and math.isnan(y)), (name, i, j, x, y)
    print(name, len(rows), "frames: legacy numeric parity (traditional/execution tolerance 1e-12, others exact)")
