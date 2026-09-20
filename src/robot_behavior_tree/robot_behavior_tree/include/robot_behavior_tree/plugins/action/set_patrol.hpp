#ifndef SET_PATROL_HPP_
#define SET_PATROL_HPP_
#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/sentryinfo.hpp"
#include "rm_interfaces/msg/hurt_data.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief A BT::ConditionNode that listens to a battery topic and
     * returns SUCCESS when battery is low and FAILURE otherwise
     */
    class SetPatrolAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        SetPatrolAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        SetPatrolAction() = delete;

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
                BT::InputPort<float>("max_pitch", "max_pitch"),
                BT::InputPort<float>("min_pitch", "min_pitch"),
                BT::InputPort<int>("mode", "mode"),
                BT::InputPort<int>("spin", "spin"),
                BT::InputPort<int>("patrol", "patrol"),
                BT::InputPort<int>("robot_aim", "robot_aim"),
            };
         }

    private:
        /**
         * @brief Callback function for battery topic
         * @param msg Shared pointer to rm_interfaces::msg::Sentryinfo message
         */
        void sentryinfoCallback(rm_interfaces::msg::Sentryinfo::SharedPtr msg);
        /**
         * @brief Callback function for battery topic
         * @param msg Shared pointer to rm_interfaces::msg::HurtData message
         */
        void hurtdataCallback(rm_interfaces::msg::HurtData::SharedPtr msg);

        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<rm_interfaces::msg::Sentryinfo>::SharedPtr sentryinfo_sub_;
        rclcpp::Subscription<rm_interfaces::msg::HurtData>::SharedPtr hurtdata_sub_;

        float max_pitch_;
        float min_pitch_;
        int mode;
        int spin;
        int patrol;
        int robot_aim;
        bool is_hurt;
        bool is_out_fight;
    };

} // namespace nav2_behavior_tree
#endif