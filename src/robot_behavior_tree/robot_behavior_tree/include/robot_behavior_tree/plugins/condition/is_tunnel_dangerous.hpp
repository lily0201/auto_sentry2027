//
// Created by elsa on 25-3-15.
//

#ifndef IS_TUNNEL_DANGEROUS_HPP
#define IS_TUNNEL_DANGEROUS_HPP

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class IsTunnelDangerousCondition : public BT::ConditionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        IsTunnelDangerousCondition(
            const std::string &condition_name,
            const BT::NodeConfiguration &conf);

        IsTunnelDangerousCondition() = delete;

        /**
         * @brief The main override required by a BT action
         * @return BT::NodeStatus Status of tick execution
         */
        BT::NodeStatus tick() override;

        /**
         * @brief Creates list of BT ports
         * @return BT::PortsList Containing node-specific ports
         */
        static BT::PortsList providedPorts()
        {
            return {};
        }

    private:
        rclcpp::Node::SharedPtr node_;

        ///狗洞是否有人堵
        std::vector<bool> dangerous_tunnel;
        int tunnel_index_;
    };

} // namespace nav2_behavior_tree

#endif //IS_TUNNEL_DANGEROUS_HPP
