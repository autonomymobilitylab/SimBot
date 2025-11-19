from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 
import xacro
from launch.actions import TimerAction
from launch.actions import ExecuteProcess, OpaqueFunction
import subprocess
import time


def wait_and_publish_detach(context):
    print(" Waiting for Flexley Tug model to load in Gazebo...")

    # Wait for model to exist in Gazebo
    while True:
        result = subprocess.run(
            ['ign', 'topic', '-l'],
            capture_output=True, text=True
        )
        if '/model/flexley_tug' in result.stdout:
            print("Model 'flexley_tug' detected in Gazebo!")

            # Now also ensure /detach topic exists
            while True:
                result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True)
                if '/detach' in result.stdout:
                    print(" /detach topic found! Sending detach command...")
                    subprocess.run(['ros2', 'topic', 'pub', '--once', '/detach', 'std_msgs/msg/Empty', '{}'])
                    print(" Detach message published.")
                    return []
                time.sleep(1)
        time.sleep(1)


def generate_launch_description():
    # Paths
    package_name = 'abb_flexley_description'
    pkg_share = get_package_share_directory(package_name)

    world_path = os.path.join(pkg_share, 'worlds', 'warehouse.sdf')
    robot_sdf_path = os.path.join(pkg_share, 'urdf', 'Flexley_Tug.sdf')
    model_path = pkg_share
   
    urdf_file = os.path.join(
        pkg_share,
        'urdf',
        'Flexley_Tug.urdf'
    )
    
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
     

    # Set required environment variables
    set_ign_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=model_path + '/models:' + model_path + '/worlds'
    )
    set_file_path = SetEnvironmentVariable(
        name='IGN_FILE_PATH',
        value=model_path + '/models:' + model_path + '/worlds'
    )

    # Launch Ignition Gazebo with the world
    launch_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo','-r' , world_path],
        output='screen'
    )

    # Spawn robot model
    spawn_model = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim','create',
            '-file', robot_sdf_path,
            '-name', 'flexley_tug',
            '-x', '-2.5', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )
    
    

    # Sensor bridges...
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu_scan@sensor_msgs/msg/Imu[ignition.msgs.IMU'],
        remappings=[('/imu_scan', '/imu_scan')],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    front_lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/lidar_front@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
        remappings=[('/lidar_front', '/lidar_front')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    back_lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/lidar_back@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
        remappings=[('/lidar_back', '/lidar_back')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    front_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/front_cam@sensor_msgs/msg/Image[ignition.msgs.Image'],
        remappings=[('/front_cam', '/camera_front')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    back_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/back_cam@sensor_msgs/msg/Image[ignition.msgs.Image'],
        remappings=[('/back_cam', '/camera_back')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    front_right_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/front_right_cam@sensor_msgs/msg/Image[ignition.msgs.Image'],
        remappings=[('/front_right_cam', '/camera_front_right')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    front_left_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/front_left_cam@sensor_msgs/msg/Image[ignition.msgs.Image'],
        remappings=[('/front_left_cam', '/camera_front_left')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    back_right_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/back_right_cam@sensor_msgs/msg/Image[ignition.msgs.Image'],
        remappings=[('/back_right_cam', '/camera_back_right')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    back_left_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/back_left_cam@sensor_msgs/msg/Image[ignition.msgs.Image'],
        remappings=[('/back_left_cam', '/camera_back_left')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    

    diff_drive_cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
        remappings=[('/cmd_vel', '/cmd_vel')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    diff_drive_odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
        remappings=[('/odom', '/odom')],
        parameters=[{'use_sim_time': True}],
        output='screen'
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
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
        remappings=[('/tf', '/tf')],
        parameters=[{'use_sim_time': True}],
        output='screen'
)

    detachable_state_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/detachable_joint_state@std_msgs/msg/String[ignition.msgs.StringMsg'],
    remappings=[('/detachable_joint_state', '/detachable_joint/state')],
    parameters=[{'use_sim_time': True}],
    output='screen'
)
   
    detach_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/detach@std_msgs/msg/Empty]ignition.msgs.Empty'],
    remappings=[('/detach', '/detach')],
    parameters=[{'use_sim_time': True}],
    output='screen'
)

    attach_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/attach@std_msgs/msg/Empty]ignition.msgs.Empty'],
    remappings=[('/attach', '/attach')],
    parameters=[{'use_sim_time': True}],
    output='screen'
)
    
  
   
    wait_for_detach_and_publish = TimerAction(
    period=5.0,
    actions=[OpaqueFunction(function=wait_and_publish_detach)]
)
    return LaunchDescription([
        set_ign_path,
        set_file_path,
        launch_gazebo,
        spawn_model,
        imu_bridge,
        front_lidar_bridge,
        back_lidar_bridge,
        diff_drive_cmd_vel_bridge,
        diff_drive_odom_bridge,
        joint_state_pub_bridge,
        front_camera_bridge,
        back_camera_bridge,
        front_right_camera_bridge, 
        front_left_camera_bridge, 
        back_right_camera_bridge,
        back_left_camera_bridge,
        joint_state_publisher_gui,
        robot_state_publisher,
        tf_bridge, 
        detachable_state_bridge,
        detach_bridge,
        attach_bridge,
        wait_for_detach_and_publish,
        
    ])
