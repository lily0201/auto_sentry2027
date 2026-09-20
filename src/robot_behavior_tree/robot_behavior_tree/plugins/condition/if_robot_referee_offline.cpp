//
// Created by elsa on 25-5-25.
//

#include <string>

#include "robot_behavior_tree/plugins/condition/if_robot_referee_offline.hpp"

namespace nav2_behavior_tree
{
    IfRobotRefereeOfflineCondition::IfRobotRefereeOfflineCondition(
        const std::string& condition_name,
        const BT::NodeConfiguration& conf)
        : BT::ConditionNode(condition_name, conf),
          is_part_of_referee_offline(false)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        hurtdata_sub_ = node_->create_subscription<rm_interfaces::msg::HurtData>(
            "/robot/hurtdata",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&IfRobotRefereeOfflineCondition::hurtdataCallback, this, std::placeholders::_1),
            sub_option);
    }

    BT::NodeStatus IfRobotRefereeOfflineCondition::tick()
    {
        callback_group_executor_.spin_some();

        if (is_part_of_referee_offline)
        {
            RCLCPP_INFO(node_->get_logger(), "!!!!!!!!裁判系统模块离线!!!!!!!");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

    void IfRobotRefereeOfflineCondition::hurtdataCallback(rm_interfaces::msg::HurtData::SharedPtr msg)
    {
        if(msg->armor_id != 0 && msg->hp_deduction_reason == 1)
        {
            is_part_of_referee_offline = true;
        }
        config().blackboard->set<bool>("is_part_of_referee_offline", is_part_of_referee_offline);
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IfRobotRefereeOfflineCondition>("IfRobotRefereeOffline");
}
