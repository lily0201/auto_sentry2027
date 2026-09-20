#include <string>

#include "robot_behavior_tree/plugins/condition/if_allowance_shoot_less_than_input.hpp"

namespace nav2_behavior_tree
{

    IfAllowanceShootLessThanInputCondition::IfAllowanceShootLessThanInputCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
          shoot_threshold_(100),
          is_shoot_less_(false)
    {
        getInput("shoot_threshold", shoot_threshold_);
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        shoot_sub_= node_->create_subscription<rm_interfaces::msg::Projectileallowance>(
            "/robot/projectileallowance",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&IfAllowanceShootLessThanInputCondition::allowanceshootCallback, this, std::placeholders::_1),
            sub_option);
    }

    BT::NodeStatus IfAllowanceShootLessThanInputCondition::tick()
    {
        getInput("shoot_threshold", shoot_threshold_);
        callback_group_executor_.spin_some();
        if (is_shoot_less_)
        {
            RCLCPP_INFO(node_->get_logger(), "发弹量 <= %d，供弹不足", shoot_threshold_);
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_INFO(node_->get_logger(), "发弹量>%d", shoot_threshold_);
        return BT::NodeStatus::FAILURE;
    }

    void IfAllowanceShootLessThanInputCondition::allowanceshootCallback(rm_interfaces::msg::Projectileallowance::SharedPtr msg)
    {
        is_shoot_less_ = msg->projectile_allowance_17mm <= shoot_threshold_;
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IfAllowanceShootLessThanInputCondition>("IfAllowanceShootLessThanInput");
}
