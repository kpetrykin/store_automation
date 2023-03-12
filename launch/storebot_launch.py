import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix


def generate_launch_description():
    package_dir = get_package_share_directory('store_automation')
    
    default_model_path = os.path.join(package_dir, 'resource', 'storebot.urdf')
    robot_description = pathlib.Path(default_model_path).read_text()
    
    print('DEFAULT_MODEL_PATH: ', default_model_path)
    
    default_rviz_config_path = os.path.join(package_dir, 'rviz/urdf_config.rviz')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'store.wbt'),
    )

    storebot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'storebot'},
        parameters=[
            {'robot_description': robot_description,
             'use_sim_time': True},
        ],
    )
    
    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '-0.29', '0', '0', '0', 'base_link', 'base_footprint'],
        parameters=[{'use_sim_time': True}]
    )
    
    map_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}]
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path],
        # parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='False',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        webots,
        storebot_driver,
        footprint_publisher,
        map_publisher,
        joint_state_publisher_node,
        robot_state_publisher_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])