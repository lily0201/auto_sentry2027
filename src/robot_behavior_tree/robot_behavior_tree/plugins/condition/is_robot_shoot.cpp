#include "robot_behavior_tree/plugins/condition/is_robot_shoot.hpp"

namespace nav2_behavior_tree
{

IsRobotShootCondition::IsRobotShootCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  last_count_(-1)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
      callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  allowance_sub_ = node_->create_subscription<rm_interfaces::msg::Projectileallowance>(
      "/robot/projectileallowance",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&IsRobotShootCondition::allowanceCallback, this, std::placeholders::_1),
      sub_option);

  // 初始化射击结束时刻为过去，保证初始不触发
  shoot_end_ = std::chrono::steady_clock::now() - std::chrono::seconds(10);
}

BT::NodeStatus IsRobotShootCondition::tick()
{
  callback_group_executor_.spin_some();

  const auto now = std::chrono::steady_clock::now();

  if (now < shoot_end_)
  {
    // 仍处于 1 秒延时内
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

void IsRobotShootCondition::allowanceCallback(
    rm_interfaces::msg::Projectileallowance::SharedPtr msg)
{
  int curr_count = static_cast<int>(msg->projectile_allowance_17mm);

  if (last_count_ >= 0 && curr_count < last_count_)
  {
    // 子弹减少，触发 1 秒延时
    shoot_end_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    RCLCPP_INFO(node_->get_logger(),
                "检测到射击，子弹 %d -> %d，延时 1 秒",
                last_count_, curr_count);
  }

  last_count_ = curr_count;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsRobotShootCondition>("IsRobotShoot");
}