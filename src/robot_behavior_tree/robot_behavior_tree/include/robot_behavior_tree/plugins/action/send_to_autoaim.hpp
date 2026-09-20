#ifndef SEND_TO_AUTOAIM_HPP_
#define SEND_TO_AUTOAIM_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/enemy_position.hpp"
#include "rm_interfaces/msg/enemy_positions.hpp"
#include "rm_interfaces/msg/lidarposition.hpp"
#include "rm_interfaces/msg/lidarstation.hpp"
#include "rm_interfaces/msg/hp.hpp"
// #include "rm_interfaces/srv/sentry_strategy.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class SendToAutoaimAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        SendToAutoaimAction(
           const std::string &action_name,
            const BT::NodeConfiguration &conf);

        SendToAutoaimAction() = delete;

        /**
         * @brief The main override required by a BT action
         * @return BT::NodeStatus Status of tick execution
         */
        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {};
        }

    private:
        // bool is_we_are_blue;
        // rm_interfaces::msg::Hp::SharedPtr enemy_hp;
        // rm_interfaces::msg::Lidarstation::SharedPtr lidarstation_info;
        // rm_interfaces::msg::EnemyPosition::SharedPtr enemy_position;
        //
        // rclcpp::Subscription<rm_interfaces::msg::Hp>::SharedPtr hp_sub_;
        // rclcpp::Subscription<rm_interfaces::msg::Lidarstation>::SharedPtr lidarstation_sub_;
        // rclcpp::Publisher<rm_interfaces::msg::EnemyPosition>::SharedPtr enemy_position_pub_;
        // rclcpp::Node::SharedPtr node_;
        // rclcpp::CallbackGroup::SharedPtr callback_group_;
        // rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        //
        // void lidarstationCallback(const rm_interfaces::msg::Lidarstation::ConstPtr &msg);
        // void hpCallback(const rm_interfaces::msg::Hp::ConstPtr &msg);
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
