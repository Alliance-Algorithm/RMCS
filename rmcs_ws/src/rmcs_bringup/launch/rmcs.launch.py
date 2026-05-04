import os
from typing import List, Optional

from ament_index_python.packages import PackageNotFoundError
from launch import (
    LaunchContext,
    LaunchDescription,
    LaunchDescriptionEntity,
)
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _strip_inline_comment(value):
    in_quote = None
    for index, char in enumerate(value):
        if char in ("'", '"'):
            in_quote = None if in_quote == char else char
        if char == '#' and in_quote is None:
            return value[:index].strip()
    return value.strip()


def _parse_scalar(value):
    value = _strip_inline_comment(value)
    if value in ('true', 'True'):
        return True
    if value in ('false', 'False'):
        return False
    if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
        return value[1:-1]
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value


def _read_robot_config(config_path):
    config = {}
    current_section = None
    in_ros_parameters = False
    current_list_key = None

    with open(config_path, encoding='utf-8') as config_file:
        for raw_line in config_file:
            line = raw_line.rstrip('\n')
            stripped = line.strip()
            if not stripped or stripped.startswith('#'):
                continue

            if not line.startswith(' ') and stripped.endswith(':'):
                current_section = stripped[:-1]
                in_ros_parameters = False
                current_list_key = None
                config[current_section] = {}
                continue

            if current_section is None:
                continue

            if line.startswith('  ') and not line.startswith('    '):
                in_ros_parameters = stripped == 'ros__parameters:'
                current_list_key = None
                if in_ros_parameters:
                    config[current_section]['ros__parameters'] = {}
                continue

            if in_ros_parameters and line.startswith('    ') and stripped.startswith('- '):
                if current_list_key is not None:
                    value = stripped[2:].strip()
                    config[current_section]['ros__parameters'][current_list_key].append(
                        _parse_scalar(value)
                    )
                continue

            if (
                not in_ros_parameters
                or not line.startswith('    ')
                or ':' not in stripped
            ):
                continue

            key, value = stripped.split(':', 1)
            if not value.strip():
                current_list_key = key
                config[current_section]['ros__parameters'][key] = []
                continue
            current_list_key = None
            config[current_section]['ros__parameters'][key] = _parse_scalar(value)

    return config


def _plugin_name(component_description):
    return str(component_description).split('->', 1)[0].strip()


def _has_component(config, plugin_name):
    components = (
        config.get('rmcs_executor', {})
        .get('ros__parameters', {})
        .get('components', [])
    )
    if not isinstance(components, list):
        return False
    return any(_plugin_name(component) == plugin_name for component in components)


def _existing_or_default_config(config_path, default_path, entities):
    if not config_path:
        return default_path
    if os.path.exists(config_path):
        return config_path

    entities.append(
        LogInfo(msg=f'Config file {config_path!r} not found, using {default_path!r}.')
    )
    return default_path


def _find_optional_package_share(package, context, entities):
    try:
        return FindPackageShare(package).perform(context)
    except PackageNotFoundError as exc:
        entities.append(
            LogInfo(msg=f'Optional package {package!r} not found; skipping: {exc}')
        )
    except Exception as exc:
        entities.append(
            LogInfo(
                msg=f'Optional package {package!r} cannot be resolved; skipping: {exc}'
            )
        )
    return None


def _mavros_node_params(mavros_params):
    device = mavros_params.get('device', '/dev/ttyACM0')
    baudrate = mavros_params.get('baudrate', 921600)
    return {
        'fcu_url': mavros_params.get('fcu_url', f'serial://{device}:{baudrate}'),
        'gcs_url': mavros_params.get('gcs_url', ''),
        'tgt_system': int(mavros_params.get('target_system_id', 1)),
        'tgt_component': int(mavros_params.get('target_component_id', 1)),
        'fcu_protocol': mavros_params.get('fcu_protocol', 'v2.0'),
    }


class MyLaunchDescriptionEntity(LaunchDescriptionEntity):
    def visit(
        self,
        context: 'LaunchContext',
    ) -> Optional[List['LaunchDescriptionEntity']]:
        entities = []

        robot_config = LaunchConfiguration('robot').perform(context)
        if robot_config.startswith('auto.'):
            is_automatic = True
            robot_name = robot_config[5:]
        else:
            is_automatic = False
            robot_name = robot_config

        automatic_suffix = '(automatic)' if is_automatic else ''
        rmcs_bringup_share = FindPackageShare('rmcs_bringup').perform(context)
        robot_yaml_path = os.path.join(
            rmcs_bringup_share,
            'config',
            robot_name + '.yaml',
        )

        entities.append(
            LogInfo(
                msg=(
                    f'Starting RMCS on robot {robot_config!r}'
                    f'{automatic_suffix} -> {robot_name}.yaml'
                )
            )
        )

        entities.append(
            Node(
                package='rmcs_executor',
                executable='rmcs_executor',
                parameters=[robot_yaml_path],
                respawn=True,
                respawn_delay=1.0,
                output='log',
            )
        )

        config = _read_robot_config(robot_yaml_path)

        odin_params = config.get('odin_ros_driver', {}).get('ros__parameters')
        if odin_params is not None and odin_params.get('enabled', True):
            odin_share = _find_optional_package_share(
                'odin_ros_driver',
                context,
                entities,
            )
            if odin_share is not None:
                respawn = odin_params.get('respawn', True)
                respawn_delay = float(odin_params.get('respawn_delay', 1.0))
                odin_config_default = os.path.join(
                    odin_share,
                    'config',
                    'control_command.yaml',
                )
                odin_config_file = _existing_or_default_config(
                    odin_params.get('config_file'),
                    odin_config_default,
                    entities,
                )
                node_name = odin_params.get('node_name', 'host_sdk_sample')
                entities.append(
                    Node(
                        package='odin_ros_driver',
                        executable='host_sdk_sample',
                        name=node_name,
                        parameters=[{'config_file': odin_config_file}],
                        respawn=respawn,
                        respawn_delay=respawn_delay,
                        output='screen',
                        emulate_tty=True,
                    )
                )

        mavros_params = config.get('mavros', {}).get('ros__parameters')
        if mavros_params is not None and mavros_params.get('enabled', True):
            mavros_share = _find_optional_package_share('mavros', context, entities)
            if mavros_share is not None:
                respawn = mavros_params.get('respawn', True)
                respawn_delay = float(mavros_params.get('respawn_delay', 1.0))
                pluginlists_yaml = os.path.join(
                    mavros_share,
                    'launch',
                    'px4_pluginlists.yaml',
                )
                px4_config_yaml = os.path.join(
                    mavros_share,
                    'launch',
                    'px4_config.yaml',
                )
                entities.append(
                    Node(
                        package='mavros',
                        executable='mavros_node',
                        namespace='mavros',
                        parameters=[
                            pluginlists_yaml,
                            px4_config_yaml,
                            _mavros_node_params(mavros_params),
                        ],
                        respawn=respawn,
                        respawn_delay=respawn_delay,
                        output='screen',
                        emulate_tty=True,
                    )
                )

        if _has_component(config, 'rmcs::AutoAimComponent'):
            auto_aim_share = _find_optional_package_share(
                'rmcs_auto_aim_v2',
                context,
                entities,
            )
            if auto_aim_share is not None:
                entities.append(
                    Node(
                        package='rmcs_auto_aim_v2',
                        executable='rmcs_auto_aim_v2_runtime',
                        name='auto_aim_runtime',
                        respawn=True,
                        respawn_delay=1.0,
                        output='screen',
                        emulate_tty=True,
                    )
                )

        if is_automatic:
            pass

        return entities


def generate_launch_description():
    ld = LaunchDescription([MyLaunchDescriptionEntity()])

    return ld
