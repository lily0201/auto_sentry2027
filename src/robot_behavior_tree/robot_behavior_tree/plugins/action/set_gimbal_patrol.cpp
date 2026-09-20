//
// Created by elsa on 25-5-24.
//

#include "robot_behavior_tree/plugins/action/set_gimbal_patrol.hpp"

namespace nav2_behavior_tree
{
    SetGimbalPatrolAction::SetGimbalPatrolAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
          patrol_(0)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

        getInput("patrol", patrol_);
    }

    BT::NodeStatus SetGimbalPatrolAction::tick()
    {
        getInput("patrol", patrol_);

        config().blackboard->set("patrol", patrol_);
        RCLCPP_INFO(node_->get_logger(), "setting gimbal patrol: %d", patrol_);

        return BT::NodeStatus::SUCCESS;
    }

} // namespace nav2_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::SetGimbalPatrolAction>("SetGimbalPatrol");
}