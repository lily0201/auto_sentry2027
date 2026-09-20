//
// Created by elsa on 25-3-27.
//

#ifndef SET_WHITELIST_HPP
#define SET_WHITELIST_HPP

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class SetWhitelistAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        SetWhitelistAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        SetWhitelistAction() = delete;

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
            return {
                BT::InputPort<int>("hero", "whitelist hero"),
                BT::InputPort<int>("engineer", "whitelist engineer"),
                BT::InputPort<int>("infantry3", "whitelist infantry3"),
                BT::InputPort<int>("infantry4", "whitelist infantry4"),
                BT::InputPort<int>("sentry", "whitelist sentry"),
                BT::InputPort<int>("outpost", "whitelist outpost"),
                BT::InputPort<int>("base", "whitelist base")
            };
        }

    private:
        /**
         * @brief Callback function for battery topic
         * @param msg Shared pointer to sensor_msgs::msg::BatteryState message
         */
        rclcpp::Node::SharedPtr node_;

        int whitelist[12];
    };

} // namespace nav2_behavior_tree

#endif //SET_WHITELIST_HPP
