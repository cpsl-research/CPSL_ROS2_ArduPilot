from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile

pkg_share = get_package_share_directory('aurelia_x6_mavlink')

ARGUMENTS = [
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
]


def generate_launch_description():
    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = LaunchConfiguration('gcs_url')

    mavros_params = ParameterFile(
        os.path.join(pkg_share, 'config', 'mavros_params.yaml'),
        allow_substs=True,
    )

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[
            mavros_params,
            {'fcu_url': fcu_url, 'gcs_url': gcs_url},
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    # The executable creates router, UAS and plugin nodes with their own names.
    # A global __node remap renames all of them and breaks private endpoints.
    ld.add_action(RegisterEventHandler(OnProcessExit(
        target_action=mavros_node,
        on_exit=[EmitEvent(event=Shutdown(reason='MAVROS process exited'))],
    )))
    ld.add_action(mavros_node)
    return ld
