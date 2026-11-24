from dataclasses import dataclass
from typing import Any, Dict, List, Optional
import os
import yaml

from launch import (
    LaunchContext,
    LaunchDescription,
    LaunchDescriptionEntity,
)
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


@dataclass(frozen=True)
class _ArmMoveitSpec:
    robot_name: str
    moveit_package: str
    urdf_xacro: str
    srdf: str
    kinematics: str
    joint_limits: str
    pilz_cartesian_limits: Optional[str]
    planning_pipelines: List[str]
    default_planning_pipeline: str
    demo_launch_file: str
    demo_launch_arguments: Dict[str, str]


def _format_launch_arg(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _load_robot_config(config_path: str) -> Dict[str, Any]:
    with open(config_path, "r", encoding="utf-8") as file:
        data = yaml.safe_load(file) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Expected YAML mapping at root of '{config_path}'")
    return data


def _extract_arm_moveit_spec(
    config_data: Dict[str, Any], config_path: str, context: LaunchContext
) -> Optional[_ArmMoveitSpec]:
    bringup_section = config_data.get("rmcs_bringup")
    if bringup_section is None:
        return None
    if not isinstance(bringup_section, dict):
        raise ValueError(f"Expected mapping under 'rmcs_bringup' in '{config_path}'")

    params = bringup_section.get("ros__parameters") or {}
    if not isinstance(params, dict):
        raise ValueError(
            f"Expected mapping under 'rmcs_bringup.ros__parameters' in '{config_path}'"
        )

    arm_type = str(params.get("arm_type") or "").strip()
    moveit_package = str(params.get("arm_moveit_package") or "").strip()
    if not arm_type and not moveit_package:
        return None

    if not moveit_package:
        moveit_package = f"{arm_type}_moveit_config"

    robot_name = str(params.get("arm_robot_name") or "").strip()
    if not robot_name:
        if not arm_type:
            raise ValueError(
                f"'{config_path}': 'arm_robot_name' is required when 'arm_moveit_package' is set without 'arm_type'"
            )
        robot_name = f"{arm_type}_description"

    moveit_pkg_share = FindPackageShare(moveit_package).perform(context)
    explicit_pilz = params.get("pilz_cartesian_limits")
    default_pilz = "config/pilz_cartesian_limits.yaml"
    if explicit_pilz is not None:
        pilz_cartesian_limits: Optional[str] = str(explicit_pilz) or None
    elif os.path.isfile(os.path.join(moveit_pkg_share, default_pilz)):
        pilz_cartesian_limits = default_pilz
    else:
        pilz_cartesian_limits = None

    raw_demo_args = params.get("demo_launch_arguments") or {}
    if not isinstance(raw_demo_args, dict):
        raise ValueError(
            f"Expected mapping under 'rmcs_bringup.ros__parameters.demo_launch_arguments' in '{config_path}'"
        )

    return _ArmMoveitSpec(
        robot_name=robot_name,
        moveit_package=moveit_package,
        urdf_xacro=str(params.get("urdf_xacro") or f"config/{robot_name}.urdf.xacro"),
        srdf=str(params.get("srdf") or f"config/{robot_name}.srdf"),
        kinematics=str(params.get("kinematics") or "config/kinematics.yaml"),
        joint_limits=str(params.get("joint_limits") or "config/joint_limits.yaml"),
        pilz_cartesian_limits=pilz_cartesian_limits,
        planning_pipelines=list(
            params.get("planning_pipelines")
            or ["ompl", "chomp", "pilz_industrial_motion_planner"]
        ),
        default_planning_pipeline=str(
            params.get("default_planning_pipeline") or "ompl"
        ),
        demo_launch_file=str(params.get("demo_launch_file") or "demo.launch.py"),
        demo_launch_arguments={
            str(k): _format_launch_arg(v) for k, v in raw_demo_args.items()
        },
    )


def _build_moveit_config_dict(spec: _ArmMoveitSpec) -> Dict[str, Any]:
    builder = (
        MoveItConfigsBuilder(spec.robot_name, package_name=spec.moveit_package)
        .robot_description(file_path=spec.urdf_xacro)
        .robot_description_semantic(file_path=spec.srdf)
        .robot_description_kinematics(file_path=spec.kinematics)
        .planning_pipelines(
            pipelines=spec.planning_pipelines,
            default_planning_pipeline=spec.default_planning_pipeline,
        )
        .joint_limits(file_path=spec.joint_limits)
    )
    if spec.pilz_cartesian_limits is not None:
        builder = builder.pilz_cartesian_limits(file_path=spec.pilz_cartesian_limits)
    return builder.to_moveit_configs().to_dict()


class MyLaunchDescriptionEntity(LaunchDescriptionEntity):
    def visit(
        self, context: "LaunchContext"
    ) -> Optional[List["LaunchDescriptionEntity"]]:
        entities = []

        robot_config = LaunchConfiguration("robot").perform(context)
        if robot_config.startswith("auto."):
            is_automatic = True
            robot_name = robot_config[5:]
        else:
            is_automatic = False
            robot_name = robot_config

        robot_yaml_path = os.path.join(
            FindPackageShare("rmcs_bringup").perform(context),
            "config",
            robot_name + ".yaml",
        )
        entities.append(
            LogInfo(
                msg=f"Starting RMCS on robot '{robot_config}'{'(automatic)' if is_automatic else ''} -> {robot_name}.yaml"
            )
        )

        config_data = _load_robot_config(robot_yaml_path)
        arm_spec = _extract_arm_moveit_spec(config_data, robot_yaml_path, context)

        executor_parameters: List[Any] = [robot_yaml_path]
        if arm_spec is not None:
            entities.append(
                LogInfo(
                    msg=f"Loading arm MoveIt config: robot='{arm_spec.robot_name}', package='{arm_spec.moveit_package}'"
                )
            )
            executor_parameters.append(_build_moveit_config_dict(arm_spec))

        entities.append(
            Node(
                package="rmcs_executor",
                executable="rmcs_executor",
                parameters=executor_parameters,
                respawn=True,
                respawn_delay=1.0,
                output="screen",
                arguments=["--ros-args", "--log-level", "rmcs_executor:=warn"],
            )
        )

        if arm_spec is not None:
            demo_launch_path = os.path.join(
                FindPackageShare(arm_spec.moveit_package).perform(context),
                "launch",
                arm_spec.demo_launch_file,
            )
            demo_arguments = {
                **arm_spec.demo_launch_arguments,
                "use_rviz": LaunchConfiguration("use_rviz"),
            }
            entities.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(demo_launch_path),
                    launch_arguments=demo_arguments.items(),
                )
            )

        return entities


def generate_launch_description():
    ld = LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="false"),
            MyLaunchDescriptionEntity(),
        ]
    )

    return ld
