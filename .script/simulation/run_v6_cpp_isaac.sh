#!/usr/bin/env bash
# Host entry point; the development container runs only the simulation bridge.
set -euo pipefail

task_repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)
task_python=${RMCS_ISAAC_PYTHON:-"${HOME}/isaacsim60-venv/bin/python"}
task_training_root=${RMCS_TRAINING_REPO:-"${task_repository_root}/../robot_rl/isaac_wheeled_rl_schedule"}
task_container=${RMCS_SIM_CONTAINER:-rmcs-rmcs-develop-1}
task_container_root=${RMCS_CONTAINER_ROOT:-/workspaces/RMCS}
task_build_jobs=${RMCS_SIM_BUILD_JOBS:-2}
task_skip_build=${RMCS_SIM_SKIP_BUILD:-0}
task_output=""
task_seconds=""
task_blend_seconds="0.0"
task_forward_args=()
task_bridge_process=""
task_bridge_started=0

usage() {
    cat <<'HELP'
Usage: .script/simulation/run_v6_cpp_isaac.sh [--headless] [--cases CASE ...]
       [--initial-profiles PROFILE ...]
       [--seconds SECONDS] [--blend-seconds SECONDS] [--output DIRECTORY] [--no-build]

Run the production C++ chassis/RL components against the free-base V6 Isaac
plant. Build rmcs_rl and the standalone bridge by default. Each invocation
owns its Unix socket and bridge; existing GUI, training and TensorBoard remain
independent. Reports, trajectory.png and summary.json go into DIRECTORY.

Cases: stand, forward_stop, backward_stop, yaw_positive, yaw_negative,
       spin_negative, spin_positive, height_low, height_high, disable
--seconds shortens each case for a smoke run; shortened cases cannot pass.
--no-build uses the existing production library and bridge executable.
--blend-seconds sets V6 upright takeover torque blending (default: 0; range: 0..0.3).
  Use 0, 0.1 and 0.2 for the direct/100 ms/200 ms engineering comparison.
--initial-profiles selects episode reset endpoints: nominal, native_prepare,
  pitch_forward, pitch_backward. These are engineering takeover benches; the
  preceding native preparation script is not executed by the reset.

Environment overrides:
  RMCS_ISAAC_PYTHON            Host Python (default: ~/isaacsim60-venv/bin/python)
  RMCS_TRAINING_REPO           V6 training repository (default: sibling robot_rl repo)
  RMCS_SIM_CONTAINER           Running development container (rmcs-rmcs-develop-1)
  RMCS_CONTAINER_ROOT          Bind-mount destination (/workspaces/RMCS)
  RMCS_SIM_BUILD_JOBS          Parallel compiler jobs (2)
  RMCS_SIM_SKIP_BUILD          Set to 1 to skip builds
  RMCS_SIM_OUTPUT_ROOT         Parent of timestamped default output directories
  RMCS_ISAAC_COMPAT_DIRECTORY  Native library compatibility directory
  RMCS_ISAAC_TMP_DIRECTORY     Kit temporary directory

Examples:
  .script/simulation/run_v6_cpp_isaac.sh --headless --cases stand --seconds 1
  .script/simulation/run_v6_cpp_isaac.sh --headless
HELP
}

fail() {
    printf 'ERROR: %s\n' "$*" >&2
    exit 1
}

while (($#)); do
    case "$1" in
    -h|--help)
        usage
        exit 0
        ;;
    --no-build)
        task_skip_build=1
        shift
        ;;
    --headless)
        task_forward_args+=("$1")
        shift
        ;;
    --seconds|--output|--blend-seconds)
        [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || fail "Missing value after $1"
        if [[ "$1" == --output ]]; then
            task_output=$2
        elif [[ "$1" == --blend-seconds ]]; then
            task_blend_seconds=$2
        else
            task_seconds=$2
            task_forward_args+=("$1" "$2")
        fi
        shift 2
        ;;
    --cases)
        task_forward_args+=("$1")
        shift
        task_case_count=0
        while (($#)) && [[ "$1" != --* ]]; do
            case "$1" in
            stand|forward_stop|backward_stop|yaw_positive|yaw_negative|spin_negative|spin_positive|height_low|height_high|disable) ;;
            *) fail "Unknown simulation case: $1" ;;
            esac
            task_forward_args+=("$1")
            task_case_count=$((task_case_count + 1))
            shift
        done
        ((task_case_count > 0)) || fail "--cases requires at least one case"
        ;;
    --initial-profiles)
        task_forward_args+=("$1")
        shift
        task_profile_count=0
        while (($#)) && [[ "$1" != --* ]]; do
            case "$1" in
            nominal|native_prepare|pitch_forward|pitch_backward) ;;
            *) fail "Unknown simulation initial profile: $1" ;;
            esac
            task_forward_args+=("$1")
            task_profile_count=$((task_profile_count + 1))
            shift
        done
        ((task_profile_count > 0)) || fail "--initial-profiles requires at least one profile"
        ;;
    *)
        fail "Unknown option: $1 (use --help; socket and training root are managed by this entry point)"
        ;;
    esac
done

[[ -x "$task_python" ]] || fail "Isaac Python is not executable: $task_python"
[[ -f "$task_training_root/src/wheeled_tasks/chassis/env.py" &&
   -f "$task_training_root/contracts/v6_flat_keyboard_playback_expectations_v1.json" ]] ||
    fail "V6 training repository not found: $task_training_root"
task_training_root=$(cd "$task_training_root" && pwd -P)
[[ "$task_skip_build" == 0 || "$task_skip_build" == 1 ]] || fail "RMCS_SIM_SKIP_BUILD must be 0 or 1"
[[ "$task_build_jobs" =~ ^[1-9][0-9]*$ ]] || fail "RMCS_SIM_BUILD_JOBS must be a positive integer"
if [[ -n "$task_seconds" ]]; then
    "$task_python" -B -c 'import math, sys; value=float(sys.argv[1]); assert math.isfinite(value) and 0 < value <= 32, "seconds must be in (0,32]"' "$task_seconds"
fi
task_blend_seconds=$("$task_python" -B -c '
import math, sys
value = float(sys.argv[1])
assert math.isfinite(value) and 0 <= value <= .3, "blend-seconds must be finite and in [0,0.3]"
print(repr(value))
' "$task_blend_seconds")
command -v docker >/dev/null 2>&1 || fail "Docker is required on the host"
command -v timeout >/dev/null 2>&1 || fail "GNU timeout is required for bounded bridge cleanup"
"$task_python" -B -c 'import importlib.util; import sys; sys.exit(0 if importlib.util.find_spec("isaaclab") else "Isaac Lab is unavailable in this Python")'
[[ $(docker inspect --format '{{.State.Running}}' "$task_container") == true ]] ||
    fail "Development container is not running: $task_container"

# Ensure the Unix socket and binary belong to this checkout, not another mount.
task_mount_source=$(docker inspect "$task_container" | "$task_python" -B -c '
import json, sys
matches = [m["Source"] for m in json.load(sys.stdin)[0]["Mounts"] if m["Destination"] == sys.argv[1]]
if len(matches) != 1:
    raise SystemExit("Expected exactly one RMCS bind mount")
print(matches[0])
' "$task_container_root")
[[ $(cd "$task_mount_source" && pwd -P) == "$task_repository_root" ]] ||
    fail "Container RMCS bind mount points to another checkout: $task_mount_source"

if [[ -z "$task_output" ]]; then
    task_output_root=${RMCS_SIM_OUTPUT_ROOT:-"${task_repository_root}/docs/zh-cn/artifacts/v6_cpp_sim"}
    task_output="${task_output_root}/run_$(date -u +%Y%m%dT%H%M%SZ)_$$"
fi
task_output=$("$task_python" -B -c 'from pathlib import Path; import sys; print(Path(sys.argv[1]).expanduser().resolve())' "$task_output")
[[ ! -e "$task_output" ]] || fail "Output already exists; choose a new directory: $task_output"
mkdir -p -- "$(dirname "$task_output")"

task_bridge_directory="$task_repository_root/rmcs_ws/build/v6_component_sim_bridge"
task_container_bridge_directory="$task_container_root/rmcs_ws/build/v6_component_sim_bridge"
task_socket="$task_bridge_directory/isaac_$$.sock"
task_container_socket="$task_container_bridge_directory/isaac_$$.sock"
task_pid_file="$task_bridge_directory/isaac_$$.pid"
task_container_pid_file="$task_container_bridge_directory/isaac_$$.pid"
task_bridge_log="$task_bridge_directory/isaac_$$.bridge.log"
task_container_binary="$task_container_bridge_directory/v6_component_sim_bridge"
umask 077
mkdir -p -- "$task_bridge_directory"
[[ ! -e "$task_socket" && ! -e "$task_pid_file" ]] || fail "This invocation's socket/PID path already exists"
"$task_python" -B -c 'import sys; assert all(len(p.encode()) < 108 for p in sys.argv[1:]), "Unix socket path exceeds 107 bytes"' "$task_socket" "$task_container_socket"

cleanup() {
    task_exit_status=$?
    trap - EXIT INT TERM
    if ((task_bridge_started)); then
        # Prefer the owned peer's protocol shutdown. This never targets another socket.
        "$task_python" -B - "$task_socket" <<'PY_SHUTDOWN' || true
import json, socket, sys
try:
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as peer:
        peer.settimeout(1.)
        peer.connect(sys.argv[1])
        peer.sendall(b'{"op":"shutdown"}\n')
        response = peer.makefile("rb").readline()
        if not json.loads(response).get("ok"):
            raise RuntimeError("Bridge rejected shutdown")
except (OSError, ValueError, RuntimeError):
    pass
PY_SHUTDOWN
        # Fallback only for the PID whose exact executable AND socket we started.
        timeout 4 docker exec -i "$task_container" python3 - \
            "$task_container_pid_file" "$task_container_binary" "$task_container_socket" <<'PY_CLEANUP' || true
from pathlib import Path
import os, signal, sys, time
pid_file, binary, endpoint = sys.argv[1:]
try:
    pid = int(Path(pid_file).read_text().strip())
except (OSError, ValueError):
    raise SystemExit(0)
if pid <= 1:
    raise SystemExit("Refusing invalid bridge PID")
def owned():
    try:
        args = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
        return args[0] == os.fsencode(binary) and b"--socket" in args and args[args.index(b"--socket") + 1] == os.fsencode(endpoint)
    except (OSError, IndexError):
        return False
for _ in range(10):
    if not owned():
        raise SystemExit(0)
    time.sleep(.05)
if owned():
    os.kill(pid, signal.SIGTERM)
    time.sleep(.2)
if owned():
    os.kill(pid, signal.SIGKILL)
PY_CLEANUP
        if [[ -n "$task_bridge_process" ]]; then
            kill "$task_bridge_process" 2>/dev/null || true
            wait "$task_bridge_process" 2>/dev/null || true
        fi
        rm -f -- "$task_socket" "$task_pid_file"
    fi
    return "$task_exit_status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ "$task_skip_build" == 0 ]]; then
    printf 'Building production rmcs_rl and the standalone simulation bridge...\n'
    docker exec -i "$task_container" bash -s -- "$task_container_root" "$task_build_jobs" <<'BUILD'
set -eo pipefail
task_root=$1
task_jobs=$2
export RMCS_PATH="$task_root"
export CMAKE_BUILD_PARALLEL_LEVEL="$task_jobs"
export RMCS_SIM_BRIDGE_BUILD_JOBS="$task_jobs"
export RMCS_SIM_BRIDGE_BUILD_DIRECTORY="$task_root/rmcs_ws/build/v6_component_sim_bridge"
cd "$task_root"
bash .script/build-rmcs --packages-up-to rmcs_rl --executor sequential
bash .script/simulation/build_v6_component_sim_bridge.sh
BUILD
fi
docker exec "$task_container" test -x "$task_container_binary" ||
    fail "Simulation bridge executable is missing; run without --no-build"

printf 'Starting owned bridge; takeover blend: %s s; log: %s\n' "$task_blend_seconds" "$task_bridge_log"
docker exec -i "$task_container" bash -s -- \
    "$task_container_root" "$task_container_pid_file" "$task_container_socket" \
    "$task_blend_seconds" >"$task_bridge_log" 2>&1 <<'BRIDGE' &
set -eo pipefail
task_root=$1
task_pid_file=$2
task_socket=$3
task_blend_seconds=$4
export RMCS_SIM_BRIDGE_BUILD_DIRECTORY="$task_root/rmcs_ws/build/v6_component_sim_bridge"
printf '%s\n' "$$" > "$task_pid_file"
exec bash "$task_root/.script/simulation/run_v6_component_sim_bridge.sh" \
    --simulation-only \
    --profile "$task_root/rmcs_ws/src/rmcs_bringup/config/wheel-leg-infantry-rl.yaml" \
    --model "$task_root/rmcs_ws/src/rmcs_rl/models/wheel_leg/policy.onnx" \
    --socket "$task_socket" \
    --param "v6_takeover_blend_seconds:=$task_blend_seconds"
BRIDGE
task_bridge_process=$!
task_bridge_started=1

"$task_python" -B - "$task_socket" "$task_blend_seconds" <<'PY_READY' || fail "Bridge not ready within 10 seconds; inspect its log"
import json, math, socket, sys, time
expected_blend = float(sys.argv[2])
deadline = time.monotonic() + 10.
last_error = None
while time.monotonic() < deadline:
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as peer:
            peer.settimeout(min(.5, max(.001, deadline - time.monotonic())))
            peer.connect(sys.argv[1])
            peer.sendall(b'{"op":"hello"}\n')
            response = json.loads(peer.makefile("rb").readline())
            if not response.get("ok") or response.get("protocol") != "rmcs_v6_component_sim_v1" or response.get("simulation_only") is not True:
                raise RuntimeError("Unexpected bridge identity")
            actual_blend = response.get("parameters", {}).get("rl_controller", {}).get("v6_takeover_blend_seconds")
            if actual_blend is not None and not math.isclose(actual_blend, expected_blend, abs_tol=1e-12):
                raise RuntimeError("Bridge takeover blend parameter differs from the requested value")
            if expected_blend > 0 and (actual_blend is None or response.get("v6_takeover_blend_supported") is not True):
                raise RuntimeError("This bridge/production library does not expose V6 takeover blending; rebuild it")
            break
    except (OSError, ValueError, RuntimeError) as error:
        last_error = error
        time.sleep(min(.05, max(0., deadline - time.monotonic())))
else:
    raise SystemExit(str(last_error))
PY_READY

task_compat_directory=${RMCS_ISAAC_COMPAT_DIRECTORY:-"${HOME}/.local/lib/compat"}
task_tmp_directory=${RMCS_ISAAC_TMP_DIRECTORY:-"${HOME}/.cache/kit-tmp"}
mkdir -p -- "$task_tmp_directory"
if [[ -d "$task_compat_directory" ]]; then
    export LD_LIBRARY_PATH="$task_compat_directory${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
printf 'Running V6 Isaac; output: %s\n' "$task_output"
set +e
env OPENBLAS_NUM_THREADS=1 TMPDIR="$task_tmp_directory" OMNI_KIT_ACCEPT_EULA=YES \
    "$task_python" -B "$task_repository_root/.script/simulation/play_v6_cpp_isaac.py" \
    --training-repo "$task_training_root" --socket "$task_socket" \
    --output "$task_output" "${task_forward_args[@]}"
task_sim_status=$?
set -e
if [[ -f "$task_output/report.json" ]]; then
    "$task_python" -B "$task_repository_root/.script/simulation/summarize_v6_cpp_sim.py" "$task_output" ||
        printf 'WARNING: Summary generation failed; original run reports remain at %s\n' "$task_output" >&2
fi
printf 'Run exit code: %s; reports: %s; bridge log: %s\n' "$task_sim_status" "$task_output" "$task_bridge_log"
exit "$task_sim_status"
