#include <string>

#include "robot_behavior_tree/plugins/condition/if_in_command.hpp"

namespace nav2_behavior_tree
{
    IfInCommandCondition::IfInCommandCondition(
        const std::string& condition_name,
        const BT::NodeConfiguration& conf)
        : BT::ConditionNode(condition_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        config().blackboard->get<bool>("in_command", in_command);
    }

    BT::NodeStatus IfInCommandCondition::tick()
    {
        config().blackboard->get<bool>("in_command", in_command);
        if (in_command)
        {
            RCLCPP_INFO(node_->get_logger(), "****************当前处于云台手控制模式***************");
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_INFO(node_->get_logger(), "^^^^^^^^^^^^^^^^^^^^当前不处于云台手控制模式^^^^^^^^^^^^^^^^^^^^");
        return BT::NodeStatus::FAILURE;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IfInCommandCondition>("IfInCommand");
}
