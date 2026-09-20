//
// Created by elsa on 25-3-15.
//

#include <string>

#include "robot_behavior_tree/plugins/condition/is_tunnel_dangerous.hpp"

namespace nav2_behavior_tree
{
    IsTunnelDangerousCondition::IsTunnelDangerousCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        config().blackboard->get<int>("tunnel_index", tunnel_index_);
        config().blackboard->get<std::vector<bool>>("dangerous_tunnel", dangerous_tunnel);
    }

    BT::NodeStatus IsTunnelDangerousCondition::tick()
    {
        config().blackboard->get<int>("tunnel_index", tunnel_index_);
        config().blackboard->get<std::vector<bool>>("dangerous_tunnel", dangerous_tunnel);

        if(dangerous_tunnel[tunnel_index_-1])
        {
            RCLCPP_INFO(node_->get_logger(), "哨兵即将经过的狗洞有车堵，切换地图");
            if(tunnel_index_ == 1)
                config().blackboard->set<std::string>("tunnel_related_map", "1");
            else if(tunnel_index_ == 2)
                config().blackboard->set<std::string>("tunnel_related_map", "2");
            else if(tunnel_index_ == 3)
                config().blackboard->set<std::string>("tunnel_related_map", "3");
            else if(tunnel_index_ == 4)
                config().blackboard->set<std::string>("tunnel_related_map", "4");
            else
                config().blackboard->set<std::string>("tunnel_related_map", "0");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IsTunnelDangerousCondition>("IsTunnelDangerous");
}
