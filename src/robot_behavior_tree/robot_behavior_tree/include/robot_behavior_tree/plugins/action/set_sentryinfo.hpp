#ifndef SET_SENTRYINFO_HPP_
#define SET_SENTRYINFO_HPP_
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
    class SetSentryInfoAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        SetSentryInfoAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        SetSentryInfoAction() = delete;

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
                BT::InputPort<std::string>("info", "info"),
            };
         }

    private:
        uint16_t sender_id;
        uint16_t receiver_id;
        std::string info;
        bool is_we_are_blue_;
    };

} // namespace nav2_behavior_tree
#endif