//
// Created by elsa on 25-4-29.
//

#ifndef WAIT_FOR_INPUT_PERIOD_HPP
#define WAIT_FOR_INPUT_PERIOD_HPP

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class WaitForInputPeriodAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        WaitForInputPeriodAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        WaitForInputPeriodAction() = delete;

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
                BT::InputPort<double>("wait_time", "wait_time"),
            };
         }

    private:
        void cancel_all_goals();
        rclcpp::Node::SharedPtr node_;
        std::chrono::milliseconds bt_loop_duration_; //行为树每次循环需要的时间
        int count;
        int count_size;
        double wait_time;
        bool if_in_mode_changing;

        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
    };

} // namespace nav2_behavior_tree

#endif //WAIT_FOR_INPUT_PERIOD_HPP
