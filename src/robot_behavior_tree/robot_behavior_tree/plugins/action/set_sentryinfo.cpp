#include <string>

#include "robot_behavior_tree/plugins/action/set_sentryinfo.hpp"
namespace nav2_behavior_tree
{
    SetSentryInfoAction::SetSentryInfoAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
        is_we_are_blue_(true), sender_id(0), receiver_id(0)
    {
        is_we_are_blue_ = config().blackboard->get<bool>("is_we_are_blue");
        getInput("info",info);
    }

    BT::NodeStatus SetSentryInfoAction::tick()
    {
        getInput("info",info);
        if(is_we_are_blue_){ //只给云台手发
            receiver_id = 0x016A;
            sender_id = 107;
        }
        else{
            receiver_id = 0x0106;
            sender_id = 7;
        }
        config().blackboard->set<uint16_t>("receiver_id", receiver_id);
        config().blackboard->set<uint16_t>("sender_id", sender_id);
        config().blackboard->set<std::string>("info",info);
        return BT::NodeStatus::SUCCESS;
    }

} // namespace nav2_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::SetSentryInfoAction>("SetSentryInfo");
}