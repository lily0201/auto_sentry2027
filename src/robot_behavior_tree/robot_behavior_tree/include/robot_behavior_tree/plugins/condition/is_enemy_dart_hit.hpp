//
// Created by elsa on 25-4-11.
//

#ifndef IS_ENEMY_DART_HIT_HPP
#define IS_ENEMY_DART_HIT_HPP

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
    class IsEnemyDartHitCondition : public BT::ConditionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        IsEnemyDartHitCondition(
            const std::string &condition_name,
            const BT::NodeConfiguration &conf);

        IsEnemyDartHitCondition() = delete;

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
        rclcpp::Node::SharedPtr node_;
        bool is_dart_hit_his;
    };

} // namespace nav2_behavior_tree

#endif //IS_ENEMY_DART_HIT_HPP
