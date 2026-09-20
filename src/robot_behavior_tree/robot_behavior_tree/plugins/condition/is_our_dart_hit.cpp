//
// Created by elsa on 25-4-11.
//

#include <string>

#include "robot_behavior_tree/plugins/condition/is_our_dart_hit.hpp"

namespace nav2_behavior_tree
{

    IsOurDartHitCondition::IsOurDartHitCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        config().blackboard->get<bool>("is_dart_hit_our", is_dart_hit_our);
    }

    BT::NodeStatus IsOurDartHitCondition::tick()
    {
        config().blackboard->get<bool>("is_dart_hit_our", is_dart_hit_our);
        if (is_dart_hit_our)
        {
            RCLCPP_INFO(node_->get_logger(), "我方飞镖命中");
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_INFO(node_->get_logger(), "我方飞镖未命中");
        return BT::NodeStatus::FAILURE;
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IsOurDartHitCondition>("IsOurDartHit");
}
