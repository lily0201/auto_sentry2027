#include <string>

#include "robot_behavior_tree/plugins/decorator/user_force_success.hpp"

namespace nav2_behavior_tree
{
    UserForceSuccessDecorator::UserForceSuccessDecorator(
        const std::string& name)
        : BT::DecoratorNode(name,{})
    {
    }

    BT::NodeStatus UserForceSuccessDecorator::tick()
    {
        setStatus(BT::NodeStatus::RUNNING);

        const BT::NodeStatus child_status = child_node_->executeTick();

        if(StatusCompleted(child_status))
        {
            resetChild();
            return BT::NodeStatus::SUCCESS;
        }

        // RUNNING
        return BT::NodeStatus::SUCCESS;
    }

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::UserForceSuccessDecorator>("UserForceSuccess");
}