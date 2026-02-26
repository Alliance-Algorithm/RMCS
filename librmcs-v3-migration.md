# librmcs v3 迁移 Prompt 清单

> **参考分支**: `dev/librmcs-v3`
> **目标分支**: `deformable-infantry`（及其他机器人硬件文件）
> **目标**: 将所有 `src/hardware/*.cpp` 及相关设备驱动头文件迁移到与 `dev/librmcs-v3` 一致的调用方式

---

## 背景说明

`dev/librmcs-v3` 分支对硬件层做了以下重构：

1. **设备驱动从继承 `librmcs::device::*` 改为完全独立（standalone）实现**，不再依赖 librmcs submodule 中的 device 层
2. **CAN 数据类型变化**：`store_status(uint64_t)` → `store_status(std::span<const std::byte>)`
3. **指令生成类型变化**：DjiMotor 返回 `CanPacket8::Quarter`，LkMotor/Supercap 返回 `CanPacket8`
4. **CAN 帧构造方式变化**：`uint16_t[4]` + `std::bit_cast` → `CanPacket8{q0,q1,q2,q3}.as_bytes()` 内联构造
5. **RingBuffer 命名空间变化**：`librmcs::utility::RingBuffer` → `utility::RingBuffer`
6. **枚举值重命名**：所有 `Type::M3508` 风格 → `Type::kM3508` 风格（加前缀 `k`）
7. **CMakeLists**：C++ 标准从 C++20 升到 C++23

---

## 一、CMakeLists.txt 变更

**文件**: `rmcs_ws/src/rmcs_core/CMakeLists.txt`

### 差异对比

| 旧（deformable-infantry） | 新（dev/librmcs-v3） |
|---|---|
| `set(CMAKE_CXX_FLAGS "... -std=c++20")` | `set(CMAKE_CXX_STANDARD 23)` |
| `include_directories(${PROJECT_SOURCE_DIR}/librmcs)` | 无（通过系统安装路径） |
| `include_directories(SYSTEM "/usr/include/libusb-1.0")` | 无 |
| `target_link_libraries(... -lusb-1.0)` | 无（librmcs SDK 已打包） |

### 修改方式

将 CMakeLists 中旧的 C++ 标准设置替换为：
```cmake
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
```

删除以下行（系统安装 librmcs SDK 后不再需要）：
```cmake
include_directories(${PROJECT_SOURCE_DIR}/librmcs)
include_directories(SYSTEM "/usr/include/libusb-1.0")
target_link_libraries(${PROJECT_NAME} -lusb-1.0)
```

> **注**: librmcs SDK 需先在 Dev Container 内安装 `.deb` 包（见 CLAUDE.md），安装后头文件位于 `/usr/local/include/librmcs/`，CMakeLists 无需手动指定路径。

---

## 二、新增文件

### 2.1 `hardware/device/can_packet.hpp`

**作用**: 提供 `CanPacket8`（8字节 CAN 帧）和 `CanPacket8::Quarter`（16位槽位）抽象，替代原来的 `uint16_t[4]` + `std::bit_cast` 模式。

**关键类型**:
```cpp
namespace rmcs_core::hardware::device {

struct CanPacket8 {
    struct Quarter { uint16_t data; };       // 16-bit 有效数据槽
    struct PaddingQuarter : Quarter { ... }; // 零填充槽

    // 4个槽位构造
    CanPacket8(Quarter q0, Quarter q1, Quarter q2, Quarter q3);

    // 获取底层字节 span（必须内联使用，不可存储返回值）
    std::span<const std::byte, 8> as_bytes();
};

} // namespace rmcs_core::hardware::device
```

### 2.2 `hardware/utility/ring_buffer.hpp`

**作用**: 替代 `librmcs::utility::RingBuffer`，接口一致。

```cpp
// 旧
#include "hardware/device/impl/ring_buffer.hpp"
librmcs::utility::RingBuffer<std::byte> ring_{256};

// 新
#include "hardware/utility/ring_buffer.hpp"
utility::RingBuffer<std::byte> ring_{256};
```

### 2.3 `rmcs_utility/include/rmcs_utility/endian_promise.hpp`

**作用**: 提供 `rmcs_utility::be_int16_t`、`rmcs_utility::le_uint16_t` 等大小端类型，供新版 `dji_motor.hpp` 使用。

---

## 三、设备驱动头文件变更

### 3.1 `hardware/device/dji_motor.hpp`

**架构变化**: 从继承 `librmcs::device::DjiMotor` → 完全独立实现

#### 枚举重命名

```cpp
// 旧
device::DjiMotor::Type::GM6020
device::DjiMotor::Type::GM6020_VOLTAGE
device::DjiMotor::Type::M3508
device::DjiMotor::Type::M2006

// 新
device::DjiMotor::Type::kGM6020
device::DjiMotor::Type::kGM6020Voltage
device::DjiMotor::Type::kM3508
device::DjiMotor::Type::kM2006
```

#### store_status 变化

```cpp
// 旧（通过 can_u64() 辅助函数将 CanDataView 转为 uint64_t）
static uint64_t can_u64(const librmcs::data::CanDataView& data) { ... }
motor.store_status(can_u64(data));

// 新（直接传 span）
motor.store_status(data.can_data);
```

#### generate_command 返回类型变化

```cpp
// 旧：返回 uint16_t
uint16_t cmd = motor.generate_command();

// 新：返回 CanPacket8::Quarter
CanPacket8::Quarter cmd = motor.generate_command();
```

#### 删除的方法

- `encoder_angle()` — 已移除，可用 `angle()` 替代

---

### 3.2 `hardware/device/lk_motor.hpp`

**架构变化**: 从继承 `librmcs::device::LkMotor` → 完全独立实现

#### 枚举重命名

```cpp
// 旧 → 新
Type::MG5010E_I10  →  Type::kMG5010Ei10
Type::MG4010E_I10  →  Type::kMG4010Ei10
Type::MG6012E_I8   →  Type::kMG6012Ei8
Type::MG4005E_I10  →  Type::kMG4005Ei10
```

#### store_status 变化

```cpp
// 旧
motor.store_status(can_u64(data));  // uint64_t

// 新
motor.store_status(data.can_data);  // std::span<const std::byte>
```

#### generate_*_command 返回类型变化

```cpp
// 旧：返回 uint64_t，需要 bit_cast
auto cmd = motor.generate_velocity_command(v);
auto raw = std::bit_cast<std::array<std::byte, 8>>(cmd);
builder.can2_transmit({.can_id = 0x142, .can_data = raw});

// 新：返回 CanPacket8，直接 .as_bytes()（必须内联）
builder.can2_transmit({
    .can_id = 0x142,
    .can_data = motor.generate_velocity_command(v).as_bytes(),
});
```

---

### 3.3 `hardware/device/bmi088.hpp`

**架构变化**: 从 `using Bmi088 = librmcs::device::Bmi088` typedef → 独立实现

**接口保持不变**，含 `set_coordinate_mapping()`、`set_coordinate_mapping_tilted()`（倾斜安装用）、`update_status()`、`q0/q1/q2/q3()`、`gz()/gy()` 等。

---

### 3.4 `hardware/device/dr16.hpp`

**架构变化**: 从继承 `librmcs::device::Dr16` → 独立实现

**接口不变**：`store_status(const std::byte*, size_t)`、`update_status()`、各摇杆/按键 getter。

---

### 3.5 `hardware/device/supercap.hpp`

#### store_status 变化

```cpp
// 旧
supercap_.store_status(can_u64(data));  // uint64_t

// 新
supercap_.store_status(data.can_data);  // std::span<const std::byte>
```

#### generate_command 返回类型变化

```cpp
// 旧：返回 uint16_t
uint16_t cmd = supercap_.generate_command();

// 新：返回 CanPacket8::Quarter
CanPacket8::Quarter cmd = supercap_.generate_command();
```

---

## 四、硬件 `.cpp` 文件变更模式

适用于：`infantry.cpp`、`tunnel_infantry.cpp`、`hero.cpp`、`steering-infantry.cpp`、`steering-hero.cpp`、`deformable-infantry.cpp`

### 4.1 Include 变更

```cpp
// 删除
#include <array>
#include <bit>
#include <cstring>
#include "hardware/device/impl/ring_buffer.hpp"

// 新增
#include "hardware/device/can_packet.hpp"
#include "hardware/utility/ring_buffer.hpp"
```

### 4.2 删除 can_u64() 辅助函数

```cpp
// 删除整个函数
static uint64_t can_u64(const librmcs::data::CanDataView& data) {
    uint64_t raw;
    std::memcpy(&raw, data.can_data.data(), sizeof(raw));
    return raw;
}
```

### 4.3 CAN 接收回调简化

```cpp
// 旧
void can1_receive_callback(const librmcs::data::CanDataView& data) override {
    if (data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() < 8)
        [[unlikely]] return;
    const uint64_t raw = ClassName::can_u64(data);
    if (data.can_id == 0x201)
        motor_a_.store_status(raw);
    else if (data.can_id == 0x202)
        motor_b_.store_status(raw);
}

// 新
void can1_receive_callback(const librmcs::data::CanDataView& data) override {
    if (data.is_extended_can_id || data.is_remote_transmission)
        return;
    if (data.can_id == 0x201)
        motor_a_.store_status(data.can_data);
    else if (data.can_id == 0x202)
        motor_b_.store_status(data.can_data);
}
```

### 4.4 CAN 发送 —— DjiMotor

```cpp
// 旧：uint16_t 数组 + bit_cast
uint16_t cmds[4] = {
    motor0_.generate_command(),
    motor1_.generate_command(),
    motor2_.generate_command(),
    motor3_.generate_command(),
};
auto raw = std::bit_cast<std::array<std::byte, 8>>(cmds);
builder.can1_transmit({.can_id = 0x200, .can_data = raw});

// 新：CanPacket8 内联构造（注意：必须内联，不可拆分存储 span）
builder.can1_transmit({
    .can_id = 0x200,
    .can_data = device::CanPacket8{
        motor0_.generate_command(),
        motor1_.generate_command(),
        motor2_.generate_command(),
        motor3_.generate_command(),
    }.as_bytes(),
});
```

带零填充槽的情况：

```cpp
builder.can1_transmit({
    .can_id = 0x1FF,
    .can_data = device::CanPacket8{
        scope_motor_.generate_command(),
        device::CanPacket8::PaddingQuarter{},
        device::CanPacket8::PaddingQuarter{},
        device::CanPacket8::PaddingQuarter{},
    }.as_bytes(),
});
```

### 4.5 CAN 发送 —— LkMotor

```cpp
// 旧：bit_cast
auto cmd = lk_motor_.generate_velocity_command(vel);
auto raw = std::bit_cast<std::array<std::byte, 8>>(cmd);
builder.can2_transmit({.can_id = 0x142, .can_data = raw});

// 新：直接 .as_bytes()（内联）
builder.can2_transmit({
    .can_id = 0x142,
    .can_data = lk_motor_.generate_velocity_command(vel).as_bytes(),
});
```

同样适用于 `generate_command()`、`generate_torque_command()`、`generate_angle_command()` 等所有 LkMotor 指令方法。

### 4.6 RingBuffer 类型

```cpp
// 旧
librmcs::utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};

// 新
utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
```

---

## 五、⚠️ 常见陷阱

### 悬空 span（Dangling Span）

`CanPacket8{...}.as_bytes()` 返回的 `span` 指向**临时对象内部**，**必须内联使用**，不可存储到变量再传递：

```cpp
// ❌ 错误：临时对象被销毁，span 悬空
auto raw = device::CanPacket8{q0, q1, q2, q3}.as_bytes();
builder.can1_transmit({.can_id = 0x200, .can_data = raw}); // UB!

// ✅ 正确：内联构造
builder.can1_transmit({
    .can_id = 0x200,
    .can_data = device::CanPacket8{q0, q1, q2, q3}.as_bytes(),
});
```

### 条件分支中的 span

当 `command_update()` 包含 `if/else` 分支且不同分支需要相同的 `CanPacket8` 时，需在**每个分支内部**分别构造：

```cpp
// ❌ 错误：在分支外提前计算 raw
auto raw_wheels = device::CanPacket8{w0,w1,w2,w3}.as_bytes();
if (condition) {
    builder.can1_transmit({.can_id=0x200, .can_data=raw_wheels}); // 可能悬空
}

// ✅ 正确：在每个分支内独立构造
if (condition) {
    builder.can1_transmit({
        .can_id = 0x200,
        .can_data = device::CanPacket8{w0,w1,w2,w3}.as_bytes(),
    });
} else {
    builder.can1_transmit({
        .can_id = 0x200,
        .can_data = device::CanPacket8{w0,w1,w2,w3}.as_bytes(),
    });
}
```

---

## 六、迁移检查清单

对每个 `src/hardware/*.cpp` 文件逐项确认：

- [ ] `#include <array>` / `#include <bit>` / `#include <cstring>` 已删除（仅保留 `<atomic>` 等确实需要的）
- [ ] `#include "hardware/device/impl/ring_buffer.hpp"` 已替换为 `"hardware/utility/ring_buffer.hpp"`
- [ ] `can_u64()` 静态辅助函数已删除
- [ ] 所有 `DjiMotor::Type::M3508` 等已重命名为 `Type::kM3508` 风格
- [ ] 所有 `LkMotor::Type::MG*` 已重命名为 `Type::kMG*` 风格
- [ ] CAN 接收回调中 `data.can_data.size() < 8` 校验已删除，`can_u64()` 已替换为 `data.can_data`
- [ ] `command_update()` 中 `uint16_t[]` + `std::bit_cast` 已替换为 `CanPacket8{...}.as_bytes()` 内联
- [ ] LkMotor 指令发送已从 `bit_cast<array<byte,8>>` 改为 `.as_bytes()`
- [ ] `librmcs::utility::RingBuffer<std::byte>` 已替换为 `utility::RingBuffer<std::byte>`
- [ ] CMakeLists.txt C++ 标准已更新为 C++23

### 新文件确认

- [ ] `src/hardware/device/can_packet.hpp` 存在
- [ ] `src/hardware/utility/ring_buffer.hpp` 存在
- [ ] `rmcs_utility/include/rmcs_utility/endian_promise.hpp` 存在

### 构建验证

```bash
build-rmcs --packages-select rmcs_core
```

期望：无编译错误，无 `-lusb-1.0` 链接错误。

---

## 七、各机器人文件速查

| 文件 | 特殊注意点 |
|---|---|
| `infantry.cpp` | 单板，yaw/pitch 均为 DjiMotor GM6020 |
| `tunnel_infantry.cpp` | 单板，pitch 为 LkMotor MG4010Ei10，`0x142` 发送 |
| `hero.cpp` | 双板（TopBoard/BottomBoard），TopBoard 有外部 IMU 线程 |
| `steering-infantry.cpp` | BottomBoard 含 `can_transmission_mode` 交替发送逻辑，分支内独立构造 CanPacket8 |
| `steering-hero.cpp` | 双板，TopBoard 含 LkMotor bullet feeder/yaw/pitch，BottomBoard 含 GM6020 转向轮 |
| `deformable-infantry.cpp` | 三板，joint motors 原用 `encoder_angle()`（已移除），替换为 `angle()` |
