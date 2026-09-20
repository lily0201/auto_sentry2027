#include <string>

#include "robot_behavior_tree/plugins/condition/is_sentry_hurt.hpp"

namespace nav2_behavior_tree
{
    IsSentryHurtCondition::IsSentryHurtCondition(
        const std::string& condition_name,
        const BT::NodeConfiguration& conf)
        : BT::ConditionNode(condition_name, conf),
          is_hurt(false)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        hurtdata_sub_ = node_->create_subscription<rm_interfaces::msg::HurtData>(
            "/robot/hurtdata",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&IsSentryHurtCondition::hurtdataCallback, this, std::placeholders::_1),
            sub_option);
    }

    BT::NodeStatus IsSentryHurtCondition::tick()
    {
        callback_group_executor_.spin_some();

        if (is_hurt)
        {
            RCLCPP_INFO(node_->get_logger(), "is_hurt!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            config().blackboard->set<bool>("is_hurt", is_hurt);
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_INFO(node_->get_logger(), "is_hurt node: not hurt");
        config().blackboard->set<bool>("is_hurt", is_hurt);
        return BT::NodeStatus::FAILURE;
    }

    void IsSentryHurtCondition::hurtdataCallback(rm_interfaces::msg::HurtData::SharedPtr msg)
    {
        if(msg->hp_deduction_reason == 0)
        {
            is_hurt = true;
            RCLCPP_INFO(node_->get_logger(), "机器人处于受伤状态");
        }
        else
        {
            is_hurt = false;
            RCLCPP_INFO(node_->get_logger(), "机器人未处于受伤状态");
        }
        config().blackboard->set<bool>("is_hurt", is_hurt);
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IsSentryHurtCondition>("IsSentryHurt");
}
