# Static TF 使用指南

`static_tf` 是一个编译期静态变换树系统，用于管理机器人坐标系之间的变换关系。

## 定义变换树结构

使用 `Joint` 和 `Link` 定义树形结构，每个节点可以指定状态类型（默认为 `MonoState`，或使用 `Eigen::Isometry3d` 等 SE3 类型）：

```cpp
#include "utility/tf/static_tf.hpp"
#include <eigen3/Eigen/Geometry>

using namespace rmcs_util;

constexpr auto static_tf = Joint {
    Link<"base_link">(),
    Joint {
        Link<"gimbal_link", Eigen::Isometry3d>(),
        Joint { Link<"camera_link", Eigen::Isometry3d>() },
    },
    Joint {
        Link<"wheel_link", Eigen::Isometry3d>(),
        Joint { Link<"left_wheel", Eigen::Isometry3d>() },
        Joint { Link<"right_wheel", Eigen::Isometry3d>() },
    },
};
using RobotTf = decltype(static_tf);
```

## 设置变换状态

### 设置单个节点的变换

```cpp
// 设置平移变换
RobotTf::set_state<"gimbal_link">(
    Eigen::Translation3d{0.16, 0.0, 0.15}
);

// 设置旋转变换
RobotTf::set_state<"camera_link">(
    Eigen::AngleAxisd{0.5 * std::numbers::pi, Eigen::Vector3d::UnitZ()}
);
```

### 设置父子节点间的变换

```cpp
// 明确指定父子关系（推荐，更清晰）
RobotTf::set_state<"base_link", "gimbal_link">(
    Eigen::Translation3d{0, 0, 0.32}
);
```

## 查找变换

计算两个节点之间的变换矩阵：

```cpp
// 计算从 camera_link 到 base_link 的变换
auto tf = RobotTf::look_up<"camera_link", "base_link", Eigen::Isometry3d>();

// 使用变换
Eigen::Vector3d point_in_camera{1, 0, 0};
Eigen::Vector3d point_in_base = tf * point_in_camera;
```

## 查询节点信息

### 检查节点是否存在

```cpp
static_assert(RobotTf::contains<"camera_link">());
```

## 遍历树结构

```cpp
// 深度优先遍历所有节点
RobotTf::foreach_df_with_parent(
    []<class T>(auto parent) { 
        std::println("{} -> {}", parent, T::name); 
    }
);
```

## 实际应用示例

### 示例：设置云台变换

```cpp
// 设置 IMU 到 Pitch 的旋转变换
Tf::set_state<"imu_link", "pitch_link">(
    Eigen::AngleAxisd{+0.5 * std::numbers::pi, Eigen::Vector3d::UnitZ()}
);

// 设置云台中心到基座的平移
constexpr auto gimbal_center_height = 0.32059;
Tf::set_state<"gimbal_center_link", "base_link">(
    Eigen::Translation3d{0, 0, -gimbal_center_height}
);
```

### 示例：设置轮子位置

```cpp
constexpr auto wheel_spacing = 0.15897;
Tf::set_state<"base_link", "left_front_wheel_link">(
    Eigen::Translation3d{+1 * wheel_spacing, +1 * wheel_spacing, 0});
Tf::set_state<"base_link", "right_front_wheel_link">(
    Eigen::Translation3d{-1 * wheel_spacing, +1 * wheel_spacing, 0});
```

## 从 YAML 加载配置

如果项目支持 YAML 序列化功能，可以从配置文件加载变换：

```cpp
#include "utility/yaml/tf.hpp"
#include <yaml-cpp/yaml.h>

// 初始化所有节点状态
RobotTf::set_state<"base_link">(Eigen::Isometry3d::Identity());
RobotTf::set_state<"gimbal_link">(Eigen::Isometry3d::Identity());
RobotTf::set_state<"camera_link">(Eigen::Isometry3d::Identity());

// YAML 配置格式
constexpr auto yaml_str = R"(
    - parent: "base_link"
      child: "gimbal_link"
      t: [0.0, 0.0, 0.32]
      q: [1.0, 0.0, 0.0, 0.0]
    
    - parent: "gimbal_link"
      child: "camera_link"
      t: [0.16, 0.0, 0.15]
      q: [1.0, 0.0, 0.0, 0.0]
)";

auto yaml = YAML::Load(yaml_str);
auto result = serialize_from<RobotTf>(yaml);
if (result.has_value()) {
    // 成功加载配置
}
```