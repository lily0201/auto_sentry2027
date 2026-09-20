//
// Created by elsa on 25-4-29.
//

#include <string>

#include "robot_behavior_tree/plugins/condition/if_mode_change.hpp"

namespace nav2_behavior_tree
{
    IfModeChangeCondition::IfModeChangeCondition(
        const std::string& condition_name,
        const BT::NodeConfiguration& conf)
        : BT::ConditionNode(condition_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        config().blackboard->get<bool>("if_in_mode_changing", if_in_mode_changing);
    }

    BT::NodeStatus IfModeChangeCondition::tick()
    {
        config().blackboard->get<bool>("if_in_mode_changing", if_in_mode_changing);
        if (if_in_mode_changing)
        {
            RCLCPP_INFO(node_->get_logger(), "导航模式切换中");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IfModeChangeCondition>("IfModeChange");
}