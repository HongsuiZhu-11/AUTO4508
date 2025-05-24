import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    # Paths
    pkg_share = get_package_share_directory("p3at_description")
    description_file = os.path.join(pkg_share, "urdf", "pioneer1.urdf")
    # Ensure this points to your updated SDF
    world_file = os.path.join(pkg_share, "world", "empty.sdf")

    # Robot description via xacro
    robot_description = ParameterValue(
        Command(['xacro ', description_file]), value_type=str)

    # --- Gazebo world & robot spawn ---
    sdf_world = ExecuteProcess(
        cmd=["gz", "sim", world_file],
        output="screen"
    )
    robot = ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_sim", "create",
             "-file", description_file, "-x", "-10", "-y", "-10", "-z", "0.0", "-Y", "0.0"],
        output="screen"
    )

    # --- State publishers / RViz ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'display.rviz')],
        output='screen'
    )

    """ aria_node = Node(
        package='aria_pkg',
        executable='ariaNode',
        name='aria_node',
        output='screen',
        arguments=[
            '--remoteIsSim',
            '--remoteHost',           'localhost',
            '--remoteRobotTcpPort',   '8101',
        ],
    ) """

    sim_aria_node = Node(
        package='sim_aria_pkg',
        executable='sim_aria_node',
        name='sim_aria_node',
        output='screen'
    )

    control_node = Node(
        package='control_package',
        executable='control_node',
        name='control_node',
        output='screen',
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[
            # adjust these to match your joystick calibration
            {'deadzone': 0.05},
            {'autorepeat_rate': 20.0},
        ],
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',  # Or 'sync_slam_toolbox_node'
        name='slam_toolbox',
        output='screen',
        parameters=[
            # You'll need to create this config file
            os.path.join(pkg_share, 'config', 'mapper_params_online.yaml')
        ],
        remappings=[
            ('/scan', '/scan'),  # Remap if your lidar topic isn't /scan
            ('/odom', '/odom'),  # Remap if your odometry topic isn't /odom
        ],
    )

    # 7) Bridge ROS2 ↔ Gazebo topics
    ros_gz_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/cmd_vel_team10@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.Imu',  # Corrected Imu
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/cmd_vel_team10@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/heartbeat_team10@std_msgs/msg/Int32@gz.msgs.Int32',
            '/drive_distance@std_msgs/msg/Float32@gz.msgs.Float32',
            '/turn_angle@std_msgs/msg/Float32@gz.msgs.Float32',
            '/joy@sensor_msgs/msg/Joy@gz.msgs.Joy',
            '/fix@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        output='screen'
    )

    return LaunchDescription([
        sdf_world,
        robot,
        robot_state_publisher,
        joint_state_pub,
        rviz,
        # aria_node,
        sim_aria_node,
        control_node,
        teleop_node,
        ros_gz_bridge,
        slam_toolbox_node,
    ])
