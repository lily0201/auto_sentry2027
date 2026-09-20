//
// Created by elsa on 25-5-15.
//

#include "robot_behavior_tree/plugins/action/force_quit_command.hpp"

namespace nav2_behavior_tree
{
    ForceQuitCommandAction::ForceQuitCommandAction(
        const std::string& action_name,
        const BT::NodeConfiguration& conf)
        : BT::SyncActionNode(action_name, conf),
          in_command(false)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    }

    BT::NodeStatus ForceQuitCommandAction::tick()
    {
        in_command = false;
        config().blackboard->set<bool>("in_command", in_command);
        RCLCPP_INFO(node_->get_logger(), "退出云台手控制模式");

        return BT::NodeStatus::SUCCESS;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::ForceQuitCommandAction>("ForceQuitCommand");
}
