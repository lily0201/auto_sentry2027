#ifndef WAIT_FOR_GAME_START_HPP_
#define WAIT_FOR_GAME_START_HPP_
#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/gamestatus.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "std_msgs/msg/header.hpp"
namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class WaitForGameStartCondition : public BT::ConditionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        WaitForGameStartCondition(
            const std::string &condition_name,
            const BT::NodeConfiguration &conf);

        WaitForGameStartCondition() = delete;

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
                 BT::InputPort<int>("if_dafu", "if_dafu"),
            };
         }

    private:
        /**
         * @brief Callback function for battery topic
         * @param msg Shared pointer to sensor_msgs::msg::BatteryState message
         */
        void gamestatusCallback(rm_interfaces::msg::Gamestatus::SharedPtr msg);
        int gamestatus;
        geometry_msgs::msg::PoseStamped pose;
        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<rm_interfaces::msg::Gamestatus>::SharedPtr gamestatus_sub_;
        rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr bt_pub_;

        /// 狗洞是否有人堵
        std::vector<bool> dangerous_tunnel;

        /// 开局是否打符
        bool if_dafu;
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
