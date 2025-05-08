#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include "ament_index_cpp/get_package_share_directory.hpp"

std::vector<geometry_msgs::msg::PoseStamped> load_waypoints(const std::string &filename, rclcpp::Node::SharedPtr node) {
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    YAML::Node config = YAML::LoadFile(filename);

    for (const auto &wp : config["waypoints"]) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = node->now();
        pose.pose.position.x = wp["position"]["x"].as<double>();
        pose.pose.position.y = wp["position"]["y"].as<double>();
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = wp["orientation"]["z"].as<double>();
        pose.pose.orientation.w = wp["orientation"]["w"].as<double>();
        waypoints.push_back(pose);
    }
    return waypoints;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("action_client_node");

    auto action_client = rclcpp_action::create_client<nav2_msgs::action::ComputePathThroughPoses>(node, "compute_path_through_poses");
    auto poses_publisher = node->create_publisher<geometry_msgs::msg::PoseArray>("waypoints", rclcpp::QoS(10));

    // Wait for the action server to be available
    while (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_INFO(node->get_logger(), "Action server not available, waiting...");
    }

    // When the action server is available, send a goal
    auto goal = nav2_msgs::action::ComputePathThroughPoses::Goal();
    goal.use_start = true;
    std::string yaml_path = ament_index_cpp::get_package_share_directory("planner_playground") + "/config/waypoints.yaml";
    auto waypoints = load_waypoints(yaml_path, node);
    goal.goals = waypoints;
    goal.start = waypoints[0];
    auto future_goal_handle = action_client->async_send_goal(goal);

    // publish the waypoints to the "waypoints" topic
    auto pose_array = std::make_shared<geometry_msgs::msg::PoseArray>();
    pose_array->header.frame_id = "map";
    pose_array->header.stamp = node->now();
    for (const auto &wp : waypoints) {
        geometry_msgs::msg::Pose pose;
        pose.position = wp.pose.position;
        pose.orientation = wp.pose.orientation;
        pose_array->poses.push_back(pose);
    }
    poses_publisher->publish(*pose_array);
    rclcpp::shutdown();
    return 0;
}
