#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("action_client_node");

    auto action_client = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(node, "compute_path_to_pose");

    // Wait for the action server to be available
    while (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_INFO(node->get_logger(), "Action server not available, waiting...");
    }

    // When the action server is available, send a goal
    auto goal = nav2_msgs::action::ComputePathToPose::Goal();
    // Set your goal parameters here
    goal.use_start = true;
    goal.start.header.frame_id = "map";
    goal.start.header.stamp = node->now();
    goal.start.pose.position.x = 17.5849; 
    goal.start.pose.position.y = 17.4194;
    goal.start.pose.orientation.x = 0.0;
    goal.start.pose.orientation.y = 0.0;
    goal.start.pose.orientation.z = 0.398072;
    goal.start.pose.orientation.w = 0.917354;

    goal.goal.header.frame_id = "map";
    goal.goal.header.stamp = node->now();
    goal.goal.pose.position.x = 14.8661;
    goal.goal.pose.position.y = 16.5106;
    goal.goal.pose.orientation.x = 0.0;
    goal.goal.pose.orientation.y = 0.0;
    goal.goal.pose.orientation.z = -0.999483;
    goal.goal.pose.orientation.w = 0.0321446;
    auto future_goal_handle = action_client->async_send_goal(goal);

    rclcpp::shutdown();
    return 0;
}
