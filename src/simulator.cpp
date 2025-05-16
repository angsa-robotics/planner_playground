#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "rclcpp_action/client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nav_msgs/msg/odometry.hpp"

class SimulatorNode : public rclcpp::Node
{
public:
    SimulatorNode()
        : Node("simulator_node"),
          initial_x_(20.0),
          initial_y_(21.0)
    {
        // Initialize transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Initialize transform
        transform_.header.frame_id = "odom";
        transform_.child_frame_id = "base_footprint";
        transform_.transform.translation.x = initial_x_;
        transform_.transform.translation.y = initial_y_;
        transform_.transform.translation.z = 0.0;
        transform_.transform.rotation.w = 1.0;
        transform_.transform.rotation.x = 0.0;
        transform_.transform.rotation.y = 0.0;
        transform_.transform.rotation.z = 0.0;

        // Create interactive marker server
        marker_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("tf_marker", this);

        // Create interactive marker
        visualization_msgs::msg::InteractiveMarker int_marker;
        int_marker.header.frame_id = "odom";
        int_marker.header.stamp = this->now();
        int_marker.name = "base_footprint_marker";
        int_marker.scale = 1.0;
        int_marker.pose.position.x = initial_x_;
        int_marker.pose.position.y = initial_y_;
        int_marker.pose.orientation.w = 1.0;

        // Add move plane control
        visualization_msgs::msg::InteractiveMarkerControl move_control;
        move_control.name = "move_plane";
        move_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
        move_control.orientation.w = 1.0;
        move_control.orientation.x = 0.0;
        move_control.orientation.y = 1.0;
        move_control.orientation.z = 0.0;
        move_control.always_visible = true;
        int_marker.controls.push_back(move_control);

        // Insert marker and set feedback callback
        marker_server_->insert(int_marker, std::bind(&SimulatorNode::processFeedback, this, std::placeholders::_1));
        marker_server_->applyChanges();

        // Create timer to publish transform
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SimulatorNode::publishTransform, this));

        // Create service to trigger action client
        service_ = this->create_service<std_srvs::srv::Trigger>("trigger_action", std::bind(&SimulatorNode::triggerAction, this, std::placeholders::_1, std::placeholders::_2));

        navigate_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

        // Initialize action client
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathThroughPoses>(this, "compute_path_through_poses");
        poses_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("waypoints", rclcpp::QoS(10));

        // Create odometry publisher
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/encoders", 10);

        // Create timer to publish odometry
        odom_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SimulatorNode::publishOdometry, this));
    }

private:
    void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
    {
        transform_.transform.translation.x = feedback->pose.position.x;
        transform_.transform.translation.y = feedback->pose.position.y;
        transform_.transform.translation.z = feedback->pose.position.z;
    }

    void publishTransform()
    {
        transform_.header.stamp = this->now();
        tf_broadcaster_->sendTransform(transform_);
    }

    void publishOdometry()
    {
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_publisher_->publish(odom_msg);
    }

    void triggerAction(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            response->success = false;
            response->message = "Action server not available";
            return;
        }

        auto goal = nav2_msgs::action::ComputePathThroughPoses::Goal();
        goal.use_start = true;
        std::string yaml_path = ament_index_cpp::get_package_share_directory("planner_playground") + "/config/waypoints.yaml";
        auto waypoints = loadWaypoints(yaml_path);

        // Set the current position to the first waypoint
        transform_.transform.translation.x = waypoints[0].pose.position.x;
        transform_.transform.translation.y = waypoints[0].pose.position.y;
        transform_.transform.translation.z = waypoints[0].pose.position.z;
        transform_.transform.rotation = waypoints[0].pose.orientation;

        // Wait a bit for the transform to be published
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Send a goal to the navigate_to_pose action server with the second waypoint
        if (!navigate_to_pose_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not available");
            response->success = false;
            response->message = "NavigateToPose action server not available";
            return;
        }

        auto navigate_goal = nav2_msgs::action::NavigateToPose::Goal();
        navigate_goal.pose = waypoints[1];
        navigate_to_pose_client_->async_send_goal(navigate_goal);

        response->success = true;
        response->message = "Action triggered successfully";
    }

    std::vector<geometry_msgs::msg::PoseStamped> loadWaypoints(const std::string &filename)
    {
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        YAML::Node config = YAML::LoadFile(filename);

        for (const auto &wp : config["waypoints"])
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = this->now();
            pose.pose.position.x = wp["pose"]["x"].as<double>();
            pose.pose.position.y = wp["pose"]["y"].as<double>();
            auto yaw = wp["pose"]["yaw"].as<double>();
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            q.normalize();
            geometry_msgs::msg::Quaternion quat;
            quat.x = q.x();
            quat.y = q.y();
            quat.z = q.z();
            quat.w = q.w();
            pose.pose.orientation = quat;
            waypoints.push_back(pose);
        }
        return waypoints;
    }

    double initial_x_;
    double initial_y_;
    geometry_msgs::msg::TransformStamped transform_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Client<nav2_msgs::action::ComputePathThroughPoses>::SharedPtr action_client_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_to_pose_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimulatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
