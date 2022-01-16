#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
import launch
import os

step_detector_path = get_package_share_directory('step_detector')
rosbag_uri = os.path.join( os.getenv("HOME") , "bagsFolder", "20211214-142719-bag")
forest_file_path = os.path.join( step_detector_path, "config", "trained_step_detector_res_0.33.yaml")
rviz2_config_path = os.path.join( step_detector_path, "config", "demo_stationary_simple_environment.rviz") 


def generate_launch_description():

    ld = LaunchDescription()

    # Launching Rosbag node
    rosbag_cmd = launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '-s', 'sqlite3', rosbag_uri],
            output='screen'
    )

    rviz_cmd = launch.actions.ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2','-d', rviz2_config_path],
            output='screen'
    )

    # filter laser data to get only legs area
    laser_filter2_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            PathJoinSubstitution([
                get_package_share_directory("step_detector"),
                "config", "laser_box_filter.yaml",
            ])],
        remappings=[ ("scan", "scan_filtered"),
                     ("scan_filtered", "scan_filtered2")]
    )

    # Launching detect_leg_clusters node
    detect_leg_clusters_node = Node(
            package="step_detector",
            executable="detect_steps",
            name="detect_steps",
            parameters= [
                {"scan_topic" : "/scan_filtered2"},
                {"fixed_frame" : "laser"},
                {"forest_file" : forest_file_path},
                {"detection_threshold": 0.01},
                {"cluster_dist_euclid": 0.13},
                {"min_points_per_cluster":  3},
                {"detect_distance_frame_id": "base_link"},
                {"max_detect_distance": 0.45},
                {"use_scan_header_stamp_for_tfs": False},
                {"max_detected_clusters": 2}
            ]
    )

    # Launching detect_leg_clusters node
    plot_leg_clusters_node = Node(
            package="step_detector",
            executable="plot_steps",
            name="plot_steps",
            parameters= [
                {"marker_display_lifetime": 0.2},
                {"steps_topic_name" : "/detected_steps"},
                {"speed_dead_zone": 0.1}
            ]
    )

    # Launching joint_leg_tracker node
    joint_leg_tracker_node = Node(
        package="step_detector",
        executable="joint_leg_tracker.py",
        name="joint_leg_tracker",
        parameters=[
            {"scan_topic" : "/scan_filtered2"},
            {"fixed_frame" : "laser"},
            {"scan_frequency" : 5},
            {"display_detected_people": True},
            {"in_free_space_threshold": 0.2},
            {"dist_travelled_together_to_initiate_leg_pair": 0.02}
        ]    
    )

    ld.add_action(laser_filter2_node)
    ld.add_action(rosbag_cmd)
    ld.add_action(detect_leg_clusters_node)
    ld.add_action(plot_leg_clusters_node)
    ld.add_action(rviz_cmd) 
    return ld 
 
