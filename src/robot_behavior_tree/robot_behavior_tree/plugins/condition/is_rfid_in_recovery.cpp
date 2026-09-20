#include <string>

#include "robot_behavior_tree/plugins/condition/is_rfid_in_recovery.hpp"

namespace nav2_behavior_tree
{

    IsRfidInRecoveryCondition::IsRfidInRecoveryCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
        rfid_in_recovery(false)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        rfidpatrol_sub_ = node_->create_subscription<rm_interfaces::msg::Rfidstatus>(
            "/robot/rfidstatus",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&IsRfidInRecoveryCondition::rfidpatrolCallback, this, std::placeholders::_1),
            sub_option);
    }

    BT::NodeStatus IsRfidInRecoveryCondition::tick()
    {
        callback_group_executor_.spin_some();
        if (rfid_in_recovery)
        {
            RCLCPP_INFO(node_->get_logger(), "RFID在补给区范围内");
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_INFO(node_->get_logger(), "RFID不在补给区范围内");
        return BT::NodeStatus::FAILURE;
    }

    void IsRfidInRecoveryCondition::rfidpatrolCallback(rm_interfaces::msg::Rfidstatus::SharedPtr msg)
    {
        rfid_in_recovery = msg->inside_recovery || msg->outside_recovery;
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IsRfidInRecoveryCondition>("IsRfidInRecovery");
}
