#include <string>

#include "robot_behavior_tree/plugins/action/select_resurrection.hpp"
namespace nav2_behavior_tree
{
    SelectResurrectionAction::SelectResurrectionAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
          resurrection_(0)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

        getInput("type_of_resurrection",resurrection_);
        config().blackboard->set<uint32_t>("resurrection",resurrection_);
    }

    BT::NodeStatus SelectResurrectionAction::tick()
    {
        getInput("type_of_resurrection",resurrection_);
        RCLCPP_INFO(node_->get_logger(), "select_resurrection:选择复活方式: %d", resurrection_);
        config().blackboard->set<uint32_t>("resurrection",resurrection_);
        return BT::NodeStatus::SUCCESS;
    }

} // namespace nav2_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::SelectResurrectionAction>("SelectResurrection");
}