from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

pkg_share = get_package_share_directory('aurelia_x6_mavlink')

ARGUMENTS = [
    DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace (e.g. cpsl_uav_1)',
    ),
    DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/ttyACM0:115200',
        description='Flight controller device URL (path:baud)',
    ),
    DeclareLaunchArgument(
        'gcs_url',
        default_value='',
        description='Optional GCS UDP forward URL (e.g. udp://@14550)',
    ),
    DeclareLaunchArgument(
        'diagnostics_enable',
        default_value='true',
        description='Launch the diagnostics health-check node',
    ),
]


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace')
    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = LaunchConfiguration('gcs_url')
    diagnostics_enable = LaunchConfiguration('diagnostics_enable')

    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'mavros.launch.py')
        ),
        launch_arguments={
            'fcu_url': fcu_url,
            'gcs_url': gcs_url,
        }.items(),
    )

    bringup_group = GroupAction([
        PushRosNamespace(namespace),
        Node(
            package='aurelia_x6_mavlink',
            executable='odom_tf_node',
            name='odom_tf_node',
            output='screen',
        ),
        Node(
            package='aurelia_x6_mavlink',
            executable='diagnostics_node',
            name='diagnostics_node',
            output='screen',
            condition=IfCondition(diagnostics_enable),
        ),
    ])

    return [mavros_launch, bringup_group]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
