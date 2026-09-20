//
// Created by elsa on 25-7-21.
//

#ifndef IF_IN_DAFU_STATUS_HPP
#define IF_IN_DAFU_STATUS_HPP

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/buff.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class IfInDafuStatusCondition : public BT::ConditionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        IfInDafuStatusCondition(
            const std::string &condition_name,
            const BT::NodeConfiguration &conf);

        IfInDafuStatusCondition() = delete;

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
        void buffCallback(rm_interfaces::msg::Buff::SharedPtr msg);

        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<rm_interfaces::msg::Buff>::SharedPtr buff_sub_;
        /// controller_server参数切换client
        std::shared_ptr<rclcpp::Client<rcl_interfaces::srv::SetParameters>> controller_server_parameter_client_;

        bool if_dafu;
        bool last_dafu_mode;

        double yaw_goal_tolerance;
    };

} // namespace nav2_behavior_tree

#endif //IF_IN_DAFU_STATUS_HPP
