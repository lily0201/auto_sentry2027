//
// Created by elsa on 25-4-19.
//

#include <string>

#include "robot_behavior_tree/plugins/condition/if_recovery_bullet_less_than_input.hpp"

namespace nav2_behavior_tree
{

    IfRecoveryBulletLessThanInputCondition::IfRecoveryBulletLessThanInputCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
          recovery_bullet_threshold_(100)
    {
        getInput("recovery_bullet_threshold", recovery_bullet_threshold_);
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        config().blackboard->get<uint16_t>("recovery_bullet_to_acquire", recovery_bullet_to_acquire);
    }

    BT::NodeStatus IfRecoveryBulletLessThanInputCondition::tick()
    {
        getInput("recovery_bullet_threshold", recovery_bullet_threshold_);
        config().blackboard->get<uint16_t>("recovery_bullet_to_acquire", recovery_bullet_to_acquire);
        if (recovery_bullet_to_acquire < recovery_bullet_threshold_)
        {
            RCLCPP_INFO(node_->get_logger(), "补给区剩余可领免费发弹量不足");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IfRecoveryBulletLessThanInputCondition>("IfRecoveryBulletLessThanInput");
}
