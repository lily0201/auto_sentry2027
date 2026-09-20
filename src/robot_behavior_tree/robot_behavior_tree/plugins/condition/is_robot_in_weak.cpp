//
// Created by elsa on 25-4-11.
//

#include <string>

#include "robot_behavior_tree/plugins/condition/is_robot_in_weak.hpp"

namespace nav2_behavior_tree
{

    IsRobotInWeakCondition::IsRobotInWeakCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        config().blackboard->get<bool>("is_in_weak", is_in_weak);
    }

    BT::NodeStatus IsRobotInWeakCondition::tick()
    {
        config().blackboard->get<bool>("is_in_weak", is_in_weak);
        if (is_in_weak)
        {
            RCLCPP_INFO(node_->get_logger(), "<<<<robot in weak>>>>");
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_INFO(node_->get_logger(), "<<<<robot out weak>>>>");
        return BT::NodeStatus::FAILURE;
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IsRobotInWeakCondition>("IsRobotInWeak");
}
