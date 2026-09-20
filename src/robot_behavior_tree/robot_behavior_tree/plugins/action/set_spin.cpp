//
// Created by elsa on 25-5-24.
//

#include "robot_behavior_tree/plugins/action/set_spin.hpp"

namespace nav2_behavior_tree
{
    SetSpinAction::SetSpinAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
          spin_(0)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

        getInput("spin", spin_);
    }

    BT::NodeStatus SetSpinAction::tick()
    {
        getInput("spin", spin_);
        if (spin_ == 1) {
            config().blackboard->set("spin", spin_);
            config().blackboard->set("mode", 1);
            RCLCPP_INFO(node_->get_logger(), "setting mode: 1, spin: %d", spin_);
        }
        else {
            config().blackboard->set("spin", spin_);
            RCLCPP_INFO(node_->get_logger(), "setting spin: %d", spin_);
        }

        return BT::NodeStatus::SUCCESS;
    }

} // namespace nav2_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::SetSpinAction>("SetSpin");
}