# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Operational Rules

- **Do NOT use git commands to stage, commit, push, or reset files**
- **Do NOT use `rm` to delete files or directories**
- **Always describe what you are about to modify and wait for user confirmation before editing any file**
- **Do NOT modify any files under `rmcs_core/librmcs/`**
- **Do NOT modify any bringup config `.yaml` files other than `deformable-infantry.yaml`**

## Commands

All scripts live in `.script/` and are on `PATH` inside the Dev Container.

```bash
build-rmcs                                      # colcon build --symlink-install in rmcs_ws/
build-rmcs --packages-select rmcs_core          # build a single package
clean-rmcs                                      # delete build/ install/ log/
set-robot deformable-infantry                   # set RMCS_ROBOT_TYPE in ~/env_setup.*
launch-rmcs                                     # ros2 launch (requires set-robot first)
lsusb                                           # verify C-Board USB devices are connected
```

No test suite exists. To validate changes, run `build-rmcs` and check compiler output.

## librmcs SDK Installation (required before build)

The hardware layer depends on the **librmcs SDK** being installed system-wide inside the Dev Container. The submodule at `rmcs_core/librmcs/` is for reference only — the actual headers and library come from the installed package.

Pre-built `.deb` packages are published at https://github.com/Alliance-Algorithm/librmcs/releases.
Current required version: **v3.0.0b2**

Inside the Dev Container (Ubuntu, x86_64):

```bash
curl -LO "https://github.com/Alliance-Algorithm/librmcs/releases/download/v3.0.0b2/librmcs-sdk-3.0.0-beta.2-amd64-release.deb"
sudo dpkg -i librmcs-sdk-3.0.0-beta.2-amd64-release.deb
```

After installation the headers are in `/usr/local/include/librmcs/` and the library in `/usr/local/lib/`. The `CMakeLists.txt` does **not** need modification — the system include path picks them up automatically.

## Architecture

### Plugin System

Every functional unit is a `rmcs_executor::Component` loaded via `pluginlib`. The executor calls each component's `update()` at a fixed rate (1000 Hz for deformable-infantry). A component that issues commands uses the pattern of registering a *partner component* whose `update()` fires after the main one.

New components must be declared with `PLUGINLIB_EXPORT_CLASS` and registered in `rmcs_ws/src/rmcs_core/plugins.xml`. Active components and their ROS parameter namespaces are listed in `rmcs_bringup/config/<robot-type>.yaml`.

### Inter-Component Communication

Components share data through typed `OutputInterface<T>` / `InputInterface<T>` pointers registered by name (e.g. `/gimbal/yaw/control_velocity`). This is **not** ROS topic pub/sub — it is a direct shared-memory mechanism inside the executor. ROS topics are only used for external interfaces (referee serial, calibration subscriptions).

### Hardware Layer (`rmcs_core/src/hardware/`)

Each robot type has one `.cpp` file. Inside it, inner classes (e.g. `FrontBoard`, `BackBoard`, `TopBoard`) privately inherit `librmcs::agent::CBoard` — one per physical USB control board. Each board class:
- Overrides `can1/2_receive_callback` and `uart/dbus/imu_receive_callback` to feed device objects
- Calls `start_transmit()` in its `command_update()` to build and send a USB packet (the returned `PacketBuilder` sends automatically on destruction — **no** `TransmitBuffer` or `trigger_transmission()`)

### librmcs Submodule (`rmcs_core/librmcs/`)

Currently pinned to the **main branch** of `https://github.com/Alliance-Algorithm/librmcs`.

| Item | main branch (current) | infantry-develop (old) |
|---|---|---|
| Header | `librmcs/agent/c_board.hpp` | `librmcs/client/cboard.hpp` |
| Namespace | `librmcs::agent::CBoard` | `librmcs::client::CBoard` |
| Device selection | `CBoard(std::string_view serial_filter)` | `CBoard(int usb_pid)` |
| CAN callback arg | `const CanDataView& data` | `uint32_t, uint64_t, bool, bool, uint8_t` |
| UART callback arg | `const UartDataView& data` | `const std::byte*, uint8_t` |
| IMU callback arg | `const AccelerometerDataView& data` | `int16_t x, y, z` |
| Sending | `start_transmit()` → `PacketBuilder` (RAII) | `TransmitBuffer` + `trigger_transmission()` |
| Event loop | managed internally by `Handler` | manual `handle_events()` thread |

`librmcs/device/` (BMI088, DJI motors, DR16, LK motors) does **not** exist on main — those device drivers live in `rmcs_core/src/hardware/device/` and are maintained in this repo.

### Controller Layer (`rmcs_core/src/controller/`)

Controllers are pure `rmcs_executor::Component` classes with no USB access. They read input interfaces (sensor values, setpoints) and write output interfaces (torque/velocity commands). PID and SMC controllers are generic and reused across robot types via YAML parameter remapping.

## Code Style

C++20, `.clang-format` enforced (LLVM base, 100-column limit, 4-space indent). Pointer alignment is left (`int* p`). Private member variables use trailing underscore (`event_thread_`).
