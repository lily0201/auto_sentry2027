#ifndef FORCE_MOVE_HPP_
#define FORCE_MOVE_HPP_
#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class ForceMoveAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        ForceMoveAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        ForceMoveAction() = delete;

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
            return {BT::InputPort<float>("force_move_time", "force_move_time_"),};
         }

    private:
        /**
         * @brief Callback function for battery topic
         * @param msg Shared pointer to sensor_msgs::msg::BatteryState message
         */
        bool is_force_move_complete;
        float force_move_time_;
        int count_size_ ;
        std::chrono::milliseconds bt_loop_duration_;
        float force_v_x;
        float force_v_y;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr force_v_pub_;
        rclcpp::Node::SharedPtr node_;
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
