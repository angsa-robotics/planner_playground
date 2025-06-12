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
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class SimulatorNode : public rclcpp::Node
{
public:
    SimulatorNode()
        : Node("simulator_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        this->declare_parameter<double>("initial_x", 0.0);
        this->declare_parameter<double>("initial_y", 0.0);

        double initial_x = this->get_parameter("initial_x").as_double();
        double initial_y = this->get_parameter("initial_y").as_double();

        // Use initial_x and initial_y in the simulator logic
        RCLCPP_INFO(this->get_logger(), "Initial position: x=%f, y=%f", initial_x, initial_y);

        // Initialize transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Initialize transform
        transform_.header.frame_id = "map";
        transform_.child_frame_id = "odom";
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
        int_marker.header.frame_id = "map";
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

        // Add rotation control for yaw with visual marker
        visualization_msgs::msg::InteractiveMarkerControl rotate_control;
        rotate_control.name = "rotate_z";
        rotate_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        rotate_control.orientation.w = 0.707;
        rotate_control.orientation.x = 0.0;
        rotate_control.orientation.y = 0.707;
        rotate_control.orientation.z = 0.0;
        rotate_control.always_visible = true;
        
        // Add a visual marker for the rotation control
        visualization_msgs::msg::Marker rotation_marker;
        rotation_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        rotation_marker.scale.x = 0.1;
        rotation_marker.scale.y = 0.1;
        rotation_marker.scale.z = 0.02;
        rotation_marker.color.r = 0.0;
        rotation_marker.color.g = 0.8;
        rotation_marker.color.b = 0.0;
        rotation_marker.color.a = 0.8;
        rotation_marker.pose.position.z = 0.1;  // Offset it slightly above the move plane
        rotate_control.markers.push_back(rotation_marker);
        
        int_marker.controls.push_back(rotate_control);

        // Insert marker and set feedback callback
        marker_server_->insert(int_marker, std::bind(&SimulatorNode::processFeedback, this, std::placeholders::_1));
        marker_server_->applyChanges();

        // Create timer to publish transform
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SimulatorNode::publishTransform, this));

        // Create service to trigger action client
        service_ = this->create_service<std_srvs::srv::Trigger>("send_poses", std::bind(&SimulatorNode::triggerAction, this, std::placeholders::_1, std::placeholders::_2));

        navigate_through_poses_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(this, "navigate_through_poses");

        // Create odometry publisher
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/encoders", 10);

        // Create timer to publish odometry
        odom_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SimulatorNode::publishOdometry, this));

        // Subscriber to mission/path topic
        path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
            "mission/path", 10,
            [this](const nav_msgs::msg::Path::SharedPtr msg) {
                latest_path_ = *msg;
            });

        // New service to send poses from subscribed topic
        send_path_service_ = this->create_service<std_srvs::srv::Trigger>(
            "send_path", std::bind(&SimulatorNode::sendPathCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
    {
        // Get marker pose (map->base_footprint) - now includes both position and orientation
        geometry_msgs::msg::Pose map_to_base_footprint = feedback->pose;
        geometry_msgs::msg::TransformStamped odom_to_base_footprint;
        try {
            std::string odom_frame = "odom";
            std::string base_frame = "base_footprint";
            odom_to_base_footprint = tf_buffer_.lookupTransform(
                odom_frame, base_frame, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get odom->base_footprint: %s", ex.what());
            return;
        }
        // Convert to tf2
        tf2::Transform tf_map_base_footprint, tf_odom_base_footprint;
        tf2::fromMsg(map_to_base_footprint, tf_map_base_footprint);
        tf2::fromMsg(odom_to_base_footprint.transform, tf_odom_base_footprint);
        // Compute map->odom (this now preserves both position and orientation from the marker)
        tf2::Transform tf_map_odom = tf_map_base_footprint * tf_odom_base_footprint.inverse();
        geometry_msgs::msg::Transform map_to_odom_msg = tf2::toMsg(tf_map_odom);
        transform_.transform = map_to_odom_msg;
        
        // Update the marker server to reflect the new pose
        marker_server_->setPose("base_footprint_marker", feedback->pose);
        marker_server_->applyChanges();
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
        if (!navigate_through_poses_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "NavigateThroughPoses action server not available");
            response->success = false;
            response->message = "NavigateThroughPoses action server not available";
            return;
        }

        auto goal = nav2_msgs::action::NavigateThroughPoses::Goal();
        goal.poses.goals = waypoints;
        navigate_through_poses_client_->async_send_goal(goal);

        response->success = true;
        response->message = "Action triggered successfully";
    }

    void sendPathCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (latest_path_.poses.empty()) {
            response->success = false;
            response->message = "No path received on mission/path topic";
            return;
        }

        if (!navigate_through_poses_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "NavigateThroughPoses action server not available");
            response->success = false;
            response->message = "NavigateThroughPoses action server not available";
            return;
        }

        auto goal = nav2_msgs::action::NavigateThroughPoses::Goal();
        goal.poses.goals = latest_path_.poses;
        navigate_through_poses_client_->async_send_goal(goal);

        response->success = true;
        response->message = "Path sent successfully";
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
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr navigate_through_poses_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    nav_msgs::msg::Path latest_path_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr send_path_service_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimulatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
