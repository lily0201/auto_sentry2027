//
// Created by elsa on 25-7-20.
//

#include "robot_behavior_tree/plugins/action/set_aim_mode.hpp"

namespace nav2_behavior_tree
{
    SetAimModeAction::SetAimModeAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
          robot_aim_(0)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

        getInput("robot_aim", robot_aim_);
    }

    BT::NodeStatus SetAimModeAction::tick()
    {
        getInput("robot_aim", robot_aim_);

        config().blackboard->set("robot_aim", robot_aim_);
        RCLCPP_INFO(node_->get_logger(), "setting robot_aim: %d", robot_aim_);

        return BT::NodeStatus::SUCCESS;
    }

} // namespace nav2_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::SetAimModeAction>("SetAimMode");
}