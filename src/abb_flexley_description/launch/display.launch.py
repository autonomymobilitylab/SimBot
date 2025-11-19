from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # 1. Locate and process xacro
    urdf_file = os.path.join(
        get_package_share_directory('abb_flexley_description'),
        'urdf',
        'Flexley_Tug.urdf'
    )
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    joint_state_pub_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model'],
        remappings=[('/joint_states', '/joint_states')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[urdf_file],
        parameters=[{'use_sim_time': True}],
        output=['screen']
    )

    # 4. RViz Node
    rviz_config_file = os.path.join(
        get_package_share_directory('abb_flexley_description'),
        'rviz',
        'model.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        joint_state_pub_bridge,
        joint_state_publisher_gui,
        robot_state_publisher,
        TimerAction(
            period=2.0,
            actions=[rviz_node]
        )
    ])

