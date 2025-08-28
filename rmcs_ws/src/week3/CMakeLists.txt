cmake_minimum_required(VERSION 3.8)
project(week3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



# 设置 CPP 的语言标准和构建类型，并导出
# compile_commands.json 供 clangd 使用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_BUILD_TYPE "Release")


# 一般来说，项目是会使用上 ros2 的，如果
# 不使用 ros2 相关库可以不引入 rclcpp
find_package(rclcpp REQUIRED)
find_package(type_description_interfaces REQUIRED)

# 执行器的头文件，暴露了 component 的接口，我们的实现需要继承该接口
find_package(rmcs_executor REQUIRED)

# 自定义的一些消息类型，例如遥控器指令，机器人种类颜色等
find_package(rmcs_msgs REQUIRED)

# tf 树的实现和相关描述文件，但它是模板元特化版本的 tf
# 如果不知道 tf（transform tree）请查阅相关资料
find_package(fast_tf REQUIRED)
find_package(rmcs_description REQUIRED)



# 将下列头文件包含近本项目，使用 cmake 管理其他 ros2 包的依赖时，
# 头文件需要自己导入，相关变量的格式为 xxx_INCLUDE_DIRS
# 如果在 package.xml 导入依赖，则可以省略这些步骤，但很可能我们
# 需要在两处地方维护依赖列表，所以我推荐只在 cmake 上维护依赖，而
# 尽可能地减少 package.xml 的修改，事实上，在如今的开发流中，只有在初始化
# 项目时会修改 package.xml 的一些基础信息，后续开发则完全可以忽略这个
# 文件
include_directories(
    ${rclcpp_INCLUDE_DIRS}
    ${type_description_interfaces_INCLUDE_DIRS}
    ${rmcs_executor_INCLUDE_DIRS}
    ${rmcs_msgs_INCLUDE_DIRS}
    ${rmcs_description_INCLUDE_DIRS}
    ${fast_tf_INCLUDE_DIRS}
)


# 创建 component 的 target
add_library(
    ${COMPONENT_NAME}_component SHARED
    ${COMPONENT_SOURCE_DIR}src/component.cpp
    # 这里是 .cc\.cpp 文件
    # 他们是组件的代码实现，后续 COMPONENT_NAME 需要被添加到
    # 各个配置文件中
)
target_link_libraries(
    ${COMPONENT_NAME}_component
    # 这里是依赖，像是 opencv
    # 上面引入的自定义 ros2 包中，rmcs_msgs，
    # rmcs_description，fast_tf 只有头文件，
    # 不需要 link，只需要 link rmcs_executor
    # 自定义包的库文件名形如：xxx_LIBRARIES
    ${rmcs_executor_LIBRARIES}
    
    # 和 rclcpp
    rclcpp::rclcpp
)


# 导出组件的信息，就是依靠这个 cmake 函数
# 先 rmcs_executor 导出 plugins.xml 文件
# 而该文件便记录着组件的类型和名字
find_package(pluginlib REQUIRED)
pluginlib_export_plugin_description_file(
    rmcs_executor plugins.xml
)

# 将包含 component 的动态链接库下载到指定目录中，
# executor 会根据 plugins.xml 来搜索相关文件
install(
  TARGETS
  ${COMPONENT_NAME}_component
  DESTINATION
  lib/${PROJECT_NAME}
)


# 由 ament 进行 ros2 相关的打包
# 导出操作，是最后的收尾阶段
find_package(ament_cmake REQUIRED)
ament_package()
