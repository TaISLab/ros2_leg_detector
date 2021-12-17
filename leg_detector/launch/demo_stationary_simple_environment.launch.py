#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
import launch
import os

leg_detector_path = get_package_share_directory('leg_detector')
rosbag_uri = os.path.join( os.getenv("HOME") , "bagsFolder", "20211214-142719-bag")
forest_file_path = os.path.join( leg_detector_path, "config", "trained_leg_detector_res=0.33.yaml")
#rviz2_config_path = leg_detector_path + "/rosbag/demos/rviz/demo_stationary_simple_environment.rviz"


def generate_launch_description():

    ld = LaunchDescription()

    # Launching Rosbag node
    rosbag_cmd = launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '-s', 'sqlite3', rosbag_uri],
            output='screen'
    )

    # filter laser data to get only legs area
    laser_filter2_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            PathJoinSubstitution([
                get_package_share_directory("leg_detector"),
                "config", "laser_box_filter.yaml",
            ])],
        remappings=[ ("scan", "scan_filtered"),
                     ("scan_filtered", "scan_filtered2")]
    )

    # Launching detect_leg_clusters node
    detect_leg_clusters_node = Node(
            package="leg_detector",
            executable="detect_leg_clusters",
            name="detect_leg_clusters",
            parameters= [
                {"forest_file" : forest_file_path},
                {"scan_topic" : "/scan_filtered2"},
                {"fixed_frame" : "laser"},
            ]
    )

    ld.add_action(laser_filter2_node)
    ld.add_action(rosbag_cmd)
    ld.add_action(detect_leg_clusters_node)


    # Launching RVIZ2
    # launch.actions.ExecuteProcess(
    #     cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', rviz2_config_path],
    #     output='screen'
    # )

    # # Launching joint_leg_tracker node
    # joint_leg_tracker_node = Node(
    #     package="leg_detector",
    #     executable="joint_leg_tracker.py",
    #     name="joint_leg_tracker",
    #     parameters=[
    #         {"scan_topic" : "/scan"},
    #         {"fixed_frame" : "laser"},
    #         {"scan_frequency" : 10}
    #     ]    
    # )

    # # Launching inflated_human_scan node
    # inflated_human_scan_node = Node(
    #     package="leg_detector",
    #     executable="inflated_human_scan",
    #     name="inflated_human_scan",
    #     parameters=[
    #         {"inflation_radius" : 1.0}
    #     ]
    # )
        
    # # Launching local_occupancy_grid_mapping node
    # local_occupancy_grid_mapping_node = Node(
    #     package="leg_detector",
    #     executable="local_occupancy_grid_mapping",
    #     name="local_occupancy_grid_mapping",
    #     parameters=[
    #         {"scan_topic" : "/scan"},
    #         {"fixed_frame" : "laser"},
    #     ]    
    # )

#    ld.add_action(joint_leg_tracker_node)
#    ld.add_action(inflated_human_scan_node)
#    ld.add_action(local_occupancy_grid_mapping_node)

    return ld 
 
