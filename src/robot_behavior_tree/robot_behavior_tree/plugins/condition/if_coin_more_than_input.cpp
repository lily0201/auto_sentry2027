#include <string>

#include "robot_behavior_tree/plugins/condition/if_coin_more_than_input.hpp"

namespace nav2_behavior_tree
{

    IfCoinMoreThanInputCondition::IfCoinMoreThanInputCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
          coin_threshold_(100),
          is_coin_more_(false)
    {
        getInput("coin_threshold", coin_threshold_);
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        coin_sub_ = node_->create_subscription<rm_interfaces::msg::Projectileallowance>(
            "/robot/projectileallowance",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&IfCoinMoreThanInputCondition::coinCallback, this, std::placeholders::_1),
            sub_option);
    }

    BT::NodeStatus IfCoinMoreThanInputCondition::tick()
    {
        getInput("coin_threshold", coin_threshold_);
        callback_group_executor_.spin_some();
        if (is_coin_more_)
        {
            RCLCPP_INFO(node_->get_logger(), "金币大于%d", coin_threshold_);
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_INFO(node_->get_logger(), "金币小于等于%d", coin_threshold_);
        return BT::NodeStatus::FAILURE;
    }

    void IfCoinMoreThanInputCondition::coinCallback(rm_interfaces::msg::Projectileallowance::SharedPtr msg)
    {
        is_coin_more_ = msg->remaining_gold_coin > coin_threshold_;
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IfCoinMoreThanInputCondition>("IfCoinMoreThanInput");
}
