#ifndef IS_ROBOT_SHOOT_HPP_
#define IS_ROBOT_SHOOT_HPP_

#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/projectileallowance.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{

class IsRobotShootCondition : public BT::ConditionNode
{
public:
  IsRobotShootCondition(
      const std::string & condition_name,
      const BT::NodeConfiguration & conf);

  IsRobotShootCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() { return {}; }

private:
  void allowanceCallback(rm_interfaces::msg::Projectileallowance::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<rm_interfaces::msg::Projectileallowance>::SharedPtr allowance_sub_;

  int last_count_{-1};                       // 上一帧子弹数
  std::chrono::steady_clock::time_point shoot_end_; // 射击状态结束时刻
};

}  // namespace nav2_behavior_tree

#endif  // IS_ROBOT_SHOOT_HPP_