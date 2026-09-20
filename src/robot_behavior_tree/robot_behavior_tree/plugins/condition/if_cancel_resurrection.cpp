#include <string>

#include "robot_behavior_tree/plugins/condition/if_cancel_resurrection.hpp"

namespace nav2_behavior_tree
{
    IfCancelResurrectionCondition::IfCancelResurrectionCondition(
        const std::string& condition_name,
        const BT::NodeConfiguration& conf)
        : BT::ConditionNode(condition_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        config().blackboard->get<bool>("is_cancel_resurrection", is_cancel_resurrection);
    }

    BT::NodeStatus IfCancelResurrectionCondition::tick()
    {
        config().blackboard->get<bool>("is_cancel_resurrection", is_cancel_resurrection);
        if (is_cancel_resurrection == true)
        {
            RCLCPP_INFO(node_->get_logger(), "哨兵强制不复活");
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_INFO(node_->get_logger(), "哨兵正常可复活");
        return BT::NodeStatus::FAILURE;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IfCancelResurrectionCondition>("IfCancelResurrection");
}
