// Copyright (c) 2023 Dexory

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using std::placeholders::_1;

class TestPlannerWithRviz : public rclcpp::Node
{
public:
  TestPlannerWithRviz()
  : Node("test_planner_with_rviz")
  {
    subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10, std::bind(&TestPlannerWithRviz::goal_pose_callback, this, _1));

    callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(callback_group_, this->get_node_base_interface());

    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this, "navigate_to_pose");
  }

private:
  void goal_pose_callback(const geometry_msgs::msg::PoseStamped & msg)
  {
    goal.pose = msg;
    action_client_->async_send_goal(goal);
  }
  nav2_msgs::action::NavigateToPose::Goal goal;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>> action_client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestPlannerWithRviz>());
  rclcpp::shutdown();
  return 0;
}
