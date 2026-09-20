//
// Created by elsa on 25-7-21.
//

#include <string>

#include "robot_behavior_tree/plugins/condition/if_in_dafu_status.hpp"

namespace nav2_behavior_tree
{

    IfInDafuStatusCondition::IfInDafuStatusCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        buff_sub_ = node_->create_subscription<rm_interfaces::msg::Buff>(
            "/robot/buff",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&IfInDafuStatusCondition::buffCallback, this, std::placeholders::_1),
            sub_option);
        // 创建参数客户端
        controller_server_parameter_client_ = node_->create_client<rcl_interfaces::srv::SetParameters>(
            "/controller_server/set_parameters");
        // 等待服务可用
        while (!controller_server_parameter_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(node_->get_logger(), "local_costmap_parameter service not available, waiting...");
        }

        config().blackboard->get<bool>("if_dafu", if_dafu);
        config().blackboard->get<bool>("if_dafu", last_dafu_mode);
    }

    BT::NodeStatus IfInDafuStatusCondition::tick()
    {
        config().blackboard->get<bool>("if_dafu", if_dafu);
        callback_group_executor_.spin_some();

        if(last_dafu_mode != if_dafu)
        {
            // 构建请求
            auto controller_server_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

            rcl_interfaces::msg::Parameter controller_server_param_yaw_goal_tolerance;
            controller_server_param_yaw_goal_tolerance.name = "general_goal_checker.yaw_goal_tolerance";
            controller_server_param_yaw_goal_tolerance.value.type =
                rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
            if(if_dafu)
                controller_server_param_yaw_goal_tolerance.value.double_value = 0.1;
            else
                controller_server_param_yaw_goal_tolerance.value.double_value = 6.28;

            // 所有要修改的参数一起push_back
            controller_server_request->parameters.push_back(controller_server_param_yaw_goal_tolerance);

            // 发送异步请求
            auto controller_server_future = controller_server_parameter_client_->async_send_request(controller_server_request);

            // 等待服务响应
            if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(),
                                                   controller_server_future) == rclcpp::FutureReturnCode::SUCCESS)
            {
                // 获取服务响应
                auto result = controller_server_future.get();
                for (size_t i = 0; i < result->results.size(); ++i)
                {
                    const auto &param_result = result->results[i];
                    const auto &param = controller_server_request->parameters[i];
                    if (param_result.successful)
                    {
                        // 检查参数类型并打印相应值
                        switch (param.value.type)
                        {
                        case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
                            RCLCPP_INFO(node_->get_logger(), "controller_server_Parameter set successfully: %s = %lf",
                                        param.name.c_str(), param.value.double_value);
                            break;
                            // 可以添加其他参数类型的处理
                        default:
                            RCLCPP_INFO(node_->get_logger(), "Parameter set successfully: %s",
                                        param.name.c_str());
                            break;
                        }
                        last_dafu_mode = if_dafu;
                    }
                    else
                    {
                        RCLCPP_ERROR(node_->get_logger(), "Failed to set parameter %s. Reason: %s",
                                     param.name.c_str(), param_result.reason.c_str());
                        return BT::NodeStatus::FAILURE;
                    }
                }
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Error getting future result");
                return BT::NodeStatus::FAILURE;
            }
        }
        if (if_dafu)
        {
            RCLCPP_INFO(node_->get_logger(), "当前在打符模式");
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_INFO(node_->get_logger(), "当前不在打符模式");
        config().blackboard->set<bool>("if_dafu", if_dafu);
        return BT::NodeStatus::FAILURE;
    }

    void IfInDafuStatusCondition::buffCallback(rm_interfaces::msg::Buff::SharedPtr msg)
    {
        if(msg->defence_buff == 25)
            if_dafu = false;
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IfInDafuStatusCondition>("IfInDafuStatus");
}
