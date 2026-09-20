//
// Created by elsa on 25-2-1.
//

#ifndef IS_RFID_IN_CENTER_HPP
#define IS_RFID_IN_CENTER_HPP
#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/rfidstatus.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class IsRfidInCenterCondition : public BT::ConditionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        IsRfidInCenterCondition(
            const std::string &condition_name,
            const BT::NodeConfiguration &conf);

        IsRfidInCenterCondition() = delete;

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
        /**
         * @brief Callback function for battery topic
         * @param msg Shared pointer to sensor_msgs::msg::BatteryState message
         */
        void rfidpatrolCallback(rm_interfaces::msg::Rfidstatus::SharedPtr msg);
        bool rfid_in_center;
        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<rm_interfaces::msg::Rfidstatus>::SharedPtr rfidpatrol_sub_;
    };

} // namespace nav2_behavior_tree

#endif //NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_RFID_IN_CENTER_HPP_
