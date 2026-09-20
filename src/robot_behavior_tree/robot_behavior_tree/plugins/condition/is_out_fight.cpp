#include <string>

#include "robot_behavior_tree/plugins/condition/is_out_fight.hpp"

namespace nav2_behavior_tree
{
    IsOutFightCondition::IsOutFightCondition(
        const std::string& condition_name,
        const BT::NodeConfiguration& conf)
        : BT::ConditionNode(condition_name, conf),
          is_out_fight(false)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        sentryinfo_sub_ = node_->create_subscription<rm_interfaces::msg::Sentryinfo>(
            "/robot/sentryinfo",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&IsOutFightCondition::sentryinfoCallback, this, std::placeholders::_1),
            sub_option);
    }

    BT::NodeStatus IsOutFightCondition::tick()
    {
        callback_group_executor_.spin_some();

        if (is_out_fight)
        {
            RCLCPP_INFO(node_->get_logger(), "机器人处于脱战状态");
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_INFO(node_->get_logger(), "机器人处于战斗状态");
        return BT::NodeStatus::FAILURE;
    }

    void IsOutFightCondition::sentryinfoCallback(rm_interfaces::msg::Sentryinfo::SharedPtr msg)
    {
        is_out_fight = msg->if_out_fight;
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IsOutFightCondition>("IsOutFight");
}
