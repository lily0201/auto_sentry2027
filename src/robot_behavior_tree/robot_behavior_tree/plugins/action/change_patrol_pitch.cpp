//
// Created by elsa on 25-5-24.
//

#include "robot_behavior_tree/plugins/action/change_patrol_pitch.hpp"

namespace nav2_behavior_tree
{
    ChangePatrolPitchAction::ChangePatrolPitchAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
          max_pitch_(0.2), min_pitch_(-0.1)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

        getInput("max_pitch", max_pitch_);
        getInput("min_pitch", min_pitch_);
    }

    BT::NodeStatus ChangePatrolPitchAction::tick()
    {
        getInput("max_pitch", max_pitch_);
        getInput("min_pitch", min_pitch_);

        config().blackboard->set("max_pitch", max_pitch_);
        config().blackboard->set("min_pitch", min_pitch_);
        RCLCPP_INFO(node_->get_logger(), "setting max_pitch: %f, min_pitch: %f", max_pitch_, min_pitch_);

       return BT::NodeStatus::SUCCESS;
    }

} // namespace nav2_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::ChangePatrolPitchAction>("ChangePatrolPitch");
}