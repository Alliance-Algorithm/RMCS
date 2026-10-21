"""Create a report-local ablation, never edit production files."""
from pathlib import Path
import difflib
import hashlib
import json
here=Path(__file__).resolve().parent
source=here.parents[1]/'rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg/recovery_tuck_controller.cpp'
original=source.read_text()
assert original.count('if (!solved.valid || !solved.ready) {')==1
modified=original.replace('if (!solved.valid || !solved.ready) {','if (!solved.valid) {')
(here/'tuck_feasibility_ablation.cpp').write_text(modified)
(here/'feasibility_ablation.patch').write_text(''.join(difflib.unified_diff(original.splitlines(True),modified.splitlines(True),fromfile=str(source),tofile='report-only-ablation')))
(here/'ablation_provenance.json').write_text(json.dumps({'source':str(source),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
    'ablation_sha256':hashlib.sha256(modified.encode()).hexdigest(),
    'change':'Only suppress solved.ready rejection; return and measure the solved IK target. Does not qualify handover or implement a complete recovery fix.'},indent=2))
