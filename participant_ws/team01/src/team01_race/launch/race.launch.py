from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
def generate_launch_description():
    share=Path(get_package_share_directory('team01_race'))
    race_start=LaunchConfiguration('race_start_topic');speed=LaunchConfiguration('speed_topic')
    steering=LaunchConfiguration('steering_topic');lidar=LaunchConfiguration('lidar_topic')
    sim=LaunchConfiguration('use_sim_time');common=[str(share/'config/racing.yaml'),{'use_sim_time':sim}]
    return LaunchDescription([
        DeclareLaunchArgument('race_start_topic',default_value='/race/start/team01'),
        DeclareLaunchArgument('speed_topic',default_value='/team01/speed'),
        DeclareLaunchArgument('steering_topic',default_value='/team01/steering'),
        DeclareLaunchArgument('lidar_topic',default_value='/team01/scan'),
        DeclareLaunchArgument('use_sim_time',default_value='true'),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='team01_actuator_bridge',
             parameters=[{'config_file':str(share/'config/bridge.yaml'),'use_sim_time':sim}]),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='team01_lidar_bridge',
             arguments=['/team01/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
             remappings=[('/team01/lidar',lidar)],parameters=[{'use_sim_time':sim}]),
        Node(package='team01_race',executable='perception_node',name='team01_perception',
             parameters=common+[{'input_topic':lidar,'output_topic':'/team01/internal/scan'}]),
        Node(package='team01_race',executable='decision_node',name='team01_decision',
             parameters=common+[{'scan_topic':'/team01/internal/scan','race_start_topic':race_start,
                                 'requested_speed_topic':'/team01/internal/requested_speed',
                                 'requested_steering_topic':'/team01/internal/requested_steering',
                                 'controller_ready_topic':'/team01/internal/controller_ready'}]),
        Node(package='team01_race',executable='control_adapter_node',name='team01_control_adapter',
             parameters=common+[{'requested_speed_topic':'/team01/internal/requested_speed',
                                 'requested_steering_topic':'/team01/internal/requested_steering',
                                 'controller_ready_topic':'/team01/internal/controller_ready',
                                 'race_start_topic':race_start,'speed_topic':speed,
                                 'steering_topic':steering,'scan_topic':lidar}]),
    ])
