from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # Define the laser scan matcher node
    laser_scan_matcher_node = Node(
        package="ros2_laser_scan_matcher",
        executable="laser_scan_matcher",
        name="laser_scan_matcher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "base_frame": "base_link",
                "odom_frame": "odom_laser",
                "map_frame": "map",
                "laser_frame": "base_link",
                "laser_scan_topic": "/scan_1",
                "publish_odom": "/odom_laser",
                "laser_odom_srv_channel": "~/enable_laser_odom",
            }
        ],
    )

    # Define the service call to enable laser odometry
    enable_laser_odom = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/laser_scan_matcher/enable_laser_odom",
            "std_srvs/srv/SetBool",
            '{"data": true}',
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            # Declare all launch arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            # Launch the laser_scan_matcher node
            laser_scan_matcher_node,
            # Call the service to enable laser odometry after a short delay
            TimerAction(period=2.0, actions=[enable_laser_odom]),
        ]
    )
