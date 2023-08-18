from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.events import matches_action
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
import os


def generate_launch_description():

    # get package directory
    pkg_dir = get_package_share_directory('pkg_name_can')

    dbc_file_path = DeclareLaunchArgument(
        'dbc_file_path',
        default_value=os.path.join(pkg_dir, "config", 'DBC_FILE_NAME'))

    pkg_name_param_file = os.path.join(pkg_dir, "param", "pkg_name.param.yaml")

    pkg_name_node = Node(
        package='pkg_name_can',
        executable='pkg_name_can_node',
        output='screen',
        namespace='pkg_name_interface',
        parameters=[
            pkg_name_param_file,
            {'dbc_file': LaunchConfiguration('dbc_file_path')},
        ],
        remappings=[
            ('can_tx', '/from_can_bus'),
            ('can_rx', '/to_can_bus'),
        ],
    )

    launch_description = [
        dbc_file_path,
        pkg_name_node
    ]

    return LaunchDescription(launch_description)