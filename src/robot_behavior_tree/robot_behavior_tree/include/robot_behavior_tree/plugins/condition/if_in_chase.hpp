//
// Created by elsa on 25-7-6.
//

#ifndef IF_IN_CHASE_HPP
#define IF_IN_CHASE_HPP
#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/enemy_chasing.hpp"
#include "rm_interfaces/msg/hp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class IfInChaseCondition : public BT::ConditionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        IfInChaseCondition(
            const std::string &condition_name,
            const BT::NodeConfiguration &conf);

        IfInChaseCondition() = delete;

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
        void EnemyChasingCallback(rm_interfaces::msg::EnemyChasing::SharedPtr msg);
        void HpCallback(rm_interfaces::msg::Hp::SharedPtr msg);

        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<rm_interfaces::msg::EnemyChasing>::SharedPtr enemy_chasing_sub_;
        rclcpp::Subscription<rm_interfaces::msg::Hp>::SharedPtr hp_sub_;

        /// 储存自瞄发来的目标位置（odom_yaw系）
        geometry_msgs::msg::PoseStamped aim_goal_;

        /// 最终由决策判断得出是否在追击状态
        bool is_in_chase;
        /// 自瞄发来的是否能追击
        bool if_can_chase;
        /// 是否在打符模式
        bool if_dafu;
        bool is_we_are_blue;

        /* 追击高低阈值 */
        double chasing_higher_limit;
        double chasing_lower_limit;

        int sentry_blood;
    };

} // namespace nav2_behavior_tree

#endif //IF_IN_CHASE_HPP
