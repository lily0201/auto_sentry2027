//
// Created by elsa on 25-4-27.
//

#include <string>

#include "robot_behavior_tree/plugins/condition/if_select_go_fort.hpp"

namespace nav2_behavior_tree
{
    IfSelectGoFortCondition::IfSelectGoFortCondition(
        const std::string& condition_name,
        const BT::NodeConfiguration& conf)
        : BT::ConditionNode(condition_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        config().blackboard->get<bool>("select_if_go_fort", select_if_go_fort);
    }

    BT::NodeStatus IfSelectGoFortCondition::tick()
    {
        config().blackboard->get<bool>("select_if_go_fort", select_if_go_fort);
        if (select_if_go_fort == true)
        {
            RCLCPP_INFO(node_->get_logger(), "哨兵上堡垒");
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_INFO(node_->get_logger(), "哨兵不上堡垒");
        return BT::NodeStatus::FAILURE;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IfSelectGoFortCondition>("IfSelectGoFort");
}
