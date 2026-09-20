//
// Created by elsa on 25-3-15.
//

#ifndef CHANGE_MODE_HPP
#define CHANGE_MODE_HPP

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "rcl_interfaces/srv/set_parameters.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief 切换当前导航控制的模式，并根据不同的控制mode切换路径规划用到的参数
     */
    class ChangeModeAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        ChangeModeAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        ChangeModeAction() = delete;

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
        rclcpp::Node::SharedPtr node_;
        /* 各个节点的切换参数client */
        std::shared_ptr<rclcpp::Client<rcl_interfaces::srv::SetParameters>> controller_server_parameter_client_;
        std::shared_ptr<rclcpp::Client<rcl_interfaces::srv::SetParameters>> local_costmap_parameter_client_;
        std::shared_ptr<rclcpp::Client<rcl_interfaces::srv::SetParameters>> global_costmap_parameter_client_;
        std::shared_ptr<rclcpp::Client<rcl_interfaces::srv::SetParameters>> bt_navigator_parameter_client_;
        std::shared_ptr<rclcpp::Client<rcl_interfaces::srv::SetParameters>> behavior_server_parameter_client_;

        ///当前导航控制的模式，0--控制底盘，1--控制云台
        int decision_mode_;
        int last_decision_mode_;
        ///当前是否小陀螺
        int spin;
        ///坐标系参数
        std::string frame_id;
        ///是否在切换mode状态，每次切换mode需要有0.5s的等待车体复位时间
        bool if_in_mode_changing;

        // std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        // std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
        // std::string remote_node_name = "parameter_blackboard";
        // std::string remote_param_name = "a_double_param";

        /* 切换参数相关 */
        ///控云台导航规划坐标系
        std::string odom_frame_gimbal;
        ///控底盘导航规划坐标系
        std::string odom_frame_chassis;
        // 控云台导航参数
        double dt_ref_gimbal;
        double max_global_plan_lookahead_dist_gimbal;
        double max_vel_x_gimbal;
        double max_vel_y_gimbal;
        double max_vel_x_backwards_gimbal;
        double max_vel_theta_gimbal;
        double acc_lim_x_gimbal;
        double acc_lim_y_gimbal;
        double acc_lim_theta_gimbal;
        // 控底盘导航参数
        double dt_ref_chassis;
        double max_global_plan_lookahead_dist_chassis;
        double max_vel_x_chassis;
        double max_vel_y_chassis;
        double max_vel_x_backwards_chassis;
        double max_vel_theta_chassis;
        double acc_lim_x_chassis;
        double acc_lim_y_chassis;
        double acc_lim_theta_chassis;
        // 小陀螺导航参数
        double dt_ref_spin;
        double max_global_plan_lookahead_dist_spin;
        double max_vel_x_spin;
        double max_vel_y_spin;
        double max_vel_x_backwards_spin;
        double max_vel_theta_spin;
        double acc_lim_x_spin;
        double acc_lim_y_spin;
        double acc_lim_theta_spin;

        //实际切换参数
        double dt_ref;
        double max_global_plan_lookahead_dist;
        double max_vel_x;
        double max_vel_y;
        double max_vel_x_backwards;
        double max_vel_theta;
        double acc_lim_x;
        double acc_lim_y;
        double acc_lim_theta;
    };

} // namespace nav2_behavior_tree

#endif //CHANGE_MODE_HPP
