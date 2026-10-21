#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
docker exec rmcs_devcontainer-rmcs-develop-1 bash -lc '
cd /workspaces/RMCS
clang++ -std=c++23 -O2 -fPIC -shared -Irmcs_ws/src/rmcs_core/src \
  rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg/recovery_profile.cpp \
  rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg/recovery_tuck_controller.cpp \
  rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg/joint_reference_recovery_controller.cpp \
  rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg/closed_chain_support_observer.cpp \
  reports/wheel_leg_recovery_fix_20261022/freebody_probe.cpp -lcrypto \
  -o reports/wheel_leg_recovery_fix_20261022/libfreebody_final.so
'
python3 - <<'PY'
import hashlib
import json
import subprocess
from pathlib import Path
root = Path.cwd()
report = root / 'reports/wheel_leg_recovery_fix_20261022'
native = root / 'rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg'
paths = list(native.glob('*.cpp')) + list(native.glob('*.hpp'))
paths += [report / 'freebody_probe.cpp', report / 'probe_freebody.py', report / 'libfreebody_final.so',
          root / 'rmcs_ws/src/rmcs_bringup/config/wheel-leg-infantry-rl.yaml',
          root / 'rmcs_ws/src/rmcs_core/test/wheel_leg_recovery_tuck.cpp',
          root / 'rmcs_ws/src/rmcs_core/tools/wheel_leg_tuck_probe.cpp',
          root / 'rmcs_ws/src/rmcs_rl/models/wheel_leg/recovery_candidate.onnx',
          root / 'rmcs_ws/src/rmcs_core/config/wheel_leg/tuck_geometry_107_v1.json']
data = dict(git_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
            source_sha256={str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in paths},
            scope='Offline physics adapter, native trajectory and ONNX action mapping. No hardware.')
(report / 'build_provenance.json').write_text(json.dumps(data, indent=2) + '\n')
PY
