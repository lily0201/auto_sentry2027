//
// Created by elsa on 25-3-15.
//

#include "robot_behavior_tree/plugins/action/change_mode.hpp"

namespace nav2_behavior_tree
{
    ChangeModeAction::ChangeModeAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
          if_in_mode_changing(false), frame_id(""),
          dt_ref(0.6), max_global_plan_lookahead_dist(1.6), max_vel_x(3.0), max_vel_y(3.0), max_vel_x_backwards(0.1),
          max_vel_theta(1.7), acc_lim_x(3.0), acc_lim_y(3.0), acc_lim_theta(0.6)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        // 创建参数客户端
        controller_server_parameter_client_ = node_->create_client<rcl_interfaces::srv::SetParameters>("/controller_server/set_parameters");
        local_costmap_parameter_client_ = node_->create_client<rcl_interfaces::srv::SetParameters>("/local_costmap/local_costmap/set_parameters");
        global_costmap_parameter_client_ = node_->create_client<rcl_interfaces::srv::SetParameters>("/global_costmap/global_costmap/set_parameters");
        bt_navigator_parameter_client_ = node_->create_client<rcl_interfaces::srv::SetParameters>("/bt_navigator/set_parameters");
        behavior_server_parameter_client_ = node_->create_client<rcl_interfaces::srv::SetParameters>("/behavior_server/set_parameters");

        // 等待服务可用
        while (!controller_server_parameter_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(node_->get_logger(), "local_costmap_parameter service not available, waiting...");
        }
        while (!local_costmap_parameter_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(node_->get_logger(), "local_costmap_parameter service not available, waiting...");
        }
        while (!global_costmap_parameter_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(node_->get_logger(), "global_costmap_parameter service not available, waiting...");
        }
        while (!bt_navigator_parameter_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(node_->get_logger(), "bt_navigator_parameter service not available, waiting...");
        }
        while (!behavior_server_parameter_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(node_->get_logger(), "behavior_server_parameter service not available, waiting...");
        }

        // remote_node_name="controller_server";
        // remote_param_name="FollowPath.max_vel_x_backwards";
        // // 创建ParameterEventHandler实例
        // param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

        // // 设置回调函数，用于监听远程节点的参数变化
        // auto cb = [this](const rclcpp::Parameter & p) {
        //     RCLCPP_INFO(
        //     node_->get_logger(), "Parameter updated: %s, type: %s, value: %s",
        //     p.get_name().c_str(),
        //     p.get_type_name().c_str(),
        //     p.value_to_string().c_str());
        // };
        // // 添加参数回调，指定远程节点名称和参数名称
        // cb_handle_ = param_subscriber_->add_parameter_callback(remote_param_name, cb, remote_node_name);

        config().blackboard->get<int>("mode", decision_mode_);
        config().blackboard->get<int>("mode", last_decision_mode_);
        config().blackboard->get<int>("spin", spin);
        //坐标系
        config().blackboard->get<std::string>("odom_frame_gimbal", odom_frame_gimbal);
        config().blackboard->get<std::string>("odom_frame_chassis", odom_frame_chassis);
        // 控云台导航参数
        config().blackboard->get<double>("dt_ref_gimbal", dt_ref_gimbal);
        config().blackboard->get<double>("max_global_plan_lookahead_dist_gimbal",
                                         max_global_plan_lookahead_dist_gimbal);
        config().blackboard->get<double>("max_vel_x_gimbal", max_vel_x_gimbal);
        config().blackboard->get<double>("max_vel_y_gimbal", max_vel_y_gimbal);
        config().blackboard->get<double>("max_vel_x_backwards_gimbal", max_vel_x_backwards_gimbal);
        config().blackboard->get<double>("max_vel_theta_gimbal", max_vel_theta_gimbal);
        config().blackboard->get<double>("acc_lim_x_gimbal", acc_lim_x_gimbal);
        config().blackboard->get<double>("acc_lim_y_gimbal", acc_lim_y_gimbal);
        config().blackboard->get<double>("acc_lim_theta_gimbal", acc_lim_theta_gimbal);
        // 控底盘导航参数
        config().blackboard->get<double>("dt_ref_chassis", dt_ref_chassis);
        config().blackboard->get<double>("max_global_plan_lookahead_dist_chassis",
                                         max_global_plan_lookahead_dist_chassis);
        config().blackboard->get<double>("max_vel_x_chassis", max_vel_x_chassis);
        config().blackboard->get<double>("max_vel_y_chassis", max_vel_y_chassis);
        config().blackboard->get<double>("max_vel_x_backwards_chassis", max_vel_x_backwards_chassis);
        config().blackboard->get<double>("max_vel_theta_chassis", max_vel_theta_chassis);
        config().blackboard->get<double>("acc_lim_x_chassis", acc_lim_x_chassis);
        config().blackboard->get<double>("acc_lim_y_chassis", acc_lim_y_chassis);
        config().blackboard->get<double>("acc_lim_theta_chassis", acc_lim_theta_chassis);
        // 小陀螺导航参数
        config().blackboard->get<double>("dt_ref_spin", dt_ref_spin);
        config().blackboard->get<double>("max_global_plan_lookahead_dist_spin",
                                         max_global_plan_lookahead_dist_spin);
        config().blackboard->get<double>("max_vel_x_spin", max_vel_x_spin);
        config().blackboard->get<double>("max_vel_y_spin", max_vel_y_spin);
        config().blackboard->get<double>("max_vel_x_backwards_spin", max_vel_x_backwards_spin);
        config().blackboard->get<double>("max_vel_theta_spin", max_vel_theta_spin);
        config().blackboard->get<double>("acc_lim_x_spin", acc_lim_x_spin);
        config().blackboard->get<double>("acc_lim_y_spin", acc_lim_y_spin);
        config().blackboard->get<double>("acc_lim_theta_spin", acc_lim_theta_spin);
    }

    BT::NodeStatus ChangeModeAction::tick()
    {
        config().blackboard->get<int>("mode", decision_mode_);
        config().blackboard->get<int>("spin",spin);
        RCLCPP_INFO(node_->get_logger(), "********当前导航模式为：%d********", decision_mode_);
        RCLCPP_INFO(node_->get_logger(), "*******last_decision_mode_：%d******", last_decision_mode_);
        RCLCPP_INFO(node_->get_logger(), "*******当前是否小陀螺：%d*******", spin);
        
        if (decision_mode_ == last_decision_mode_)
        {
            return BT::NodeStatus::SUCCESS;
        }
        if(spin == 1)
        {
            RCLCPP_INFO(node_->get_logger(), "change_mode: 当前在小陀螺导航");
            last_decision_mode_ = decision_mode_;

            frame_id = odom_frame_gimbal;
            dt_ref = dt_ref_spin;
            max_global_plan_lookahead_dist = max_global_plan_lookahead_dist_spin;
            max_vel_x = max_vel_x_spin;
            max_vel_y = max_vel_y_spin;
            max_vel_x_backwards = max_vel_x_backwards_spin;
            max_vel_theta = max_vel_theta_spin;
            acc_lim_x = acc_lim_x_spin;
            acc_lim_y = acc_lim_y_spin;
            acc_lim_theta = acc_lim_theta_spin;
        }
        else if (decision_mode_ == 0)
        {
            RCLCPP_INFO(node_->get_logger(), "change_mode: 当前导航控制底盘");
            last_decision_mode_ = decision_mode_;

            frame_id = odom_frame_chassis;
            dt_ref = dt_ref_chassis;
            max_global_plan_lookahead_dist = max_global_plan_lookahead_dist_chassis;
            max_vel_x = max_vel_x_chassis;
            max_vel_y = max_vel_y_chassis;
            max_vel_x_backwards = max_vel_x_backwards_chassis;
            max_vel_theta = max_vel_theta_chassis;
            acc_lim_x = acc_lim_x_chassis;
            acc_lim_y = acc_lim_y_chassis;
            acc_lim_theta = acc_lim_theta_chassis;
        }
        else if (decision_mode_ == 1)
        {
            RCLCPP_INFO(node_->get_logger(), "change_mode: 当前导航控制云台");
            last_decision_mode_ = decision_mode_;

            frame_id = odom_frame_gimbal;
            dt_ref = dt_ref_gimbal;
            max_global_plan_lookahead_dist = max_global_plan_lookahead_dist_gimbal;
            max_vel_x = max_vel_x_gimbal;
            max_vel_y = max_vel_y_gimbal;
            max_vel_x_backwards = max_vel_x_backwards_gimbal;
            max_vel_theta = max_vel_theta_gimbal;
            acc_lim_x = acc_lim_x_gimbal;
            acc_lim_y = acc_lim_y_gimbal;
            acc_lim_theta = acc_lim_theta_gimbal;
        }

        if_in_mode_changing = true; // 进入切换模式的等待状态
        RCLCPP_INFO(node_->get_logger(), "!!!!!!!!!!!!!!!if_in_mode_changing: %d", if_in_mode_changing);
        config().blackboard->set<bool>("if_in_mode_changing", if_in_mode_changing);

        /* 修改FollowPath中的参数 */
        // 构建请求
        auto controller_server_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

        rcl_interfaces::msg::Parameter controller_server_param_dt_ref;
        controller_server_param_dt_ref.name = "FollowPath.dt_ref";
        controller_server_param_dt_ref.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        rcl_interfaces::msg::Parameter controller_server_param_max_global_plan_lookahead_dist;
        controller_server_param_max_global_plan_lookahead_dist.name = "FollowPath.max_global_plan_lookahead_dist";
        controller_server_param_max_global_plan_lookahead_dist.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        rcl_interfaces::msg::Parameter controller_server_param_max_vel_theta;
        controller_server_param_max_vel_theta.name = "FollowPath.max_vel_theta";
        controller_server_param_max_vel_theta.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        rcl_interfaces::msg::Parameter controller_server_param_max_vel_x;
        controller_server_param_max_vel_x.name = "FollowPath.max_vel_x";
        controller_server_param_max_vel_x.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        rcl_interfaces::msg::Parameter controller_server_param_max_vel_y;
        controller_server_param_max_vel_y.name = "FollowPath.max_vel_y";
        controller_server_param_max_vel_y.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        rcl_interfaces::msg::Parameter controller_server_param_max_vel_x_backwards;
        controller_server_param_max_vel_x_backwards.name = "FollowPath.max_vel_x_backwards";
        controller_server_param_max_vel_x_backwards.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        rcl_interfaces::msg::Parameter controller_server_param_acc_lim_x;
        controller_server_param_acc_lim_x.name = "FollowPath.acc_lim_x";
        controller_server_param_acc_lim_x.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        rcl_interfaces::msg::Parameter controller_server_param_acc_lim_y;
        controller_server_param_acc_lim_y.name = "FollowPath.acc_lim_y";
        controller_server_param_acc_lim_y.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        rcl_interfaces::msg::Parameter controller_server_param_acc_lim_theta;
        controller_server_param_acc_lim_theta.name = "FollowPath.acc_lim_theta";
        controller_server_param_acc_lim_theta.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;

        controller_server_param_dt_ref.value.double_value = dt_ref;
        controller_server_param_max_global_plan_lookahead_dist.value.double_value = max_global_plan_lookahead_dist;
        controller_server_param_max_vel_theta.value.double_value = max_vel_theta;
        controller_server_param_max_vel_x.value.double_value = max_vel_x;
        controller_server_param_max_vel_y.value.double_value = max_vel_y;
        controller_server_param_max_vel_x_backwards.value.double_value = max_vel_x_backwards;
        controller_server_param_acc_lim_x.value.double_value = acc_lim_x;
        controller_server_param_acc_lim_y.value.double_value = acc_lim_y;
        controller_server_param_acc_lim_theta.value.double_value = acc_lim_theta;

        // 所有要修改的参数一起push_back
        controller_server_request->parameters.push_back(controller_server_param_dt_ref);
        controller_server_request->parameters.push_back(controller_server_param_max_global_plan_lookahead_dist);
        controller_server_request->parameters.push_back(controller_server_param_max_vel_theta);
        controller_server_request->parameters.push_back(controller_server_param_max_vel_x);
        controller_server_request->parameters.push_back(controller_server_param_max_vel_y);
        controller_server_request->parameters.push_back(controller_server_param_max_vel_x_backwards);
        controller_server_request->parameters.push_back(controller_server_param_acc_lim_x);
        controller_server_request->parameters.push_back(controller_server_param_acc_lim_y);
        controller_server_request->parameters.push_back(controller_server_param_acc_lim_theta);

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
        // controller_server_
        /* 修改bt_navigator.robot_base_frame */
        // 构建请求
        auto bt_navigator_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        rcl_interfaces::msg::Parameter bt_navigator_param;
        bt_navigator_param.name = "robot_base_frame";
        bt_navigator_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        bt_navigator_param.value.string_value = frame_id;
        bt_navigator_request->parameters.push_back(bt_navigator_param);
        // 发送异步请求
        auto bt_navigator_future = bt_navigator_parameter_client_->async_send_request(bt_navigator_request);

        // 等待服务响应
        if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(),
                                               bt_navigator_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            // 获取服务响应
            auto result = bt_navigator_future.get();
            for (size_t i = 0; i < result->results.size(); ++i)
            {
                const auto &param_result = result->results[i];
                const auto &param = bt_navigator_request->parameters[i];

                if (param_result.successful)
                {
                    RCLCPP_INFO(node_->get_logger(), "bt_navigator_Parameter set successfully: %s = %s",
                                param.name.c_str(), param.value.string_value.c_str());
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
            RCLCPP_ERROR(node_->get_logger(), "Error getting future result for bt_navigator");
            return BT::NodeStatus::FAILURE;
        }

        /* local_costmap.robot_base_frame */
        // 构建请求
        auto local_costmap_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        rcl_interfaces::msg::Parameter local_costmap_param;
        local_costmap_param.name = "robot_base_frame";
        local_costmap_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        local_costmap_param.value.string_value = frame_id;
        local_costmap_request->parameters.push_back(local_costmap_param);
        // 发送异步请求
        auto local_costmap_future = local_costmap_parameter_client_->async_send_request(local_costmap_request);

        // 等待服务响应
        if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(),
                                               local_costmap_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            // 获取服务响应
            auto result = local_costmap_future.get();
            for (size_t i = 0; i < result->results.size(); ++i)
            {
                const auto &param_result = result->results[i];
                const auto &param = local_costmap_request->parameters[i];

                if (param_result.successful)
                {
                    RCLCPP_INFO(node_->get_logger(), "local_costmap_Parameter set successfully: %s = %s",
                                param.name.c_str(), param.value.string_value.c_str());
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
            RCLCPP_ERROR(node_->get_logger(), "Error getting future result for local_costmap");
            return BT::NodeStatus::FAILURE;
        }

        /* global_costmap.robot_base_frame */
        // 构建请求
        auto global_costmap_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        rcl_interfaces::msg::Parameter global_costmap_param;
        global_costmap_param.name = "robot_base_frame";
        global_costmap_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        global_costmap_param.value.string_value = frame_id;
        global_costmap_request->parameters.push_back(global_costmap_param);
        // 发送异步请求
        auto global_costmap_future = global_costmap_parameter_client_->async_send_request(global_costmap_request);

        // 等待服务响应
        if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(),
                                               global_costmap_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            // 获取服务响应
            auto result = global_costmap_future.get();
            for (size_t i = 0; i < result->results.size(); ++i)
            {
                const auto &param_result = result->results[i];
                const auto &param = global_costmap_request->parameters[i];

                if (param_result.successful)
                {
                    RCLCPP_INFO(node_->get_logger(), "global_costmap_Parameter set successfully: %s = %s",
                                param.name.c_str(), param.value.string_value.c_str());
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
            RCLCPP_ERROR(node_->get_logger(), "Error getting future result for global_costmap");
            return BT::NodeStatus::FAILURE;
        }

        /* behavior_server.robot_base_frame */
        // 构建请求
        auto behavior_server_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        rcl_interfaces::msg::Parameter behavior_server_param;
        behavior_server_param.name = "robot_base_frame";
        behavior_server_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        behavior_server_param.value.string_value = frame_id;
        behavior_server_request->parameters.push_back(behavior_server_param);
        // 发送异步请求
        auto behavior_server_future = behavior_server_parameter_client_->async_send_request(behavior_server_request);

        // 等待服务响应
        if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(),
                                               behavior_server_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            // 获取服务响应
            auto result = behavior_server_future.get();
            for (size_t i = 0; i < result->results.size(); ++i)
            {
                const auto &param_result = result->results[i];
                const auto &param = behavior_server_request->parameters[i];

                if (param_result.successful)
                {
                    RCLCPP_INFO(node_->get_logger(), "behavior_server_Parameter set successfully: %s = %s",
                                param.name.c_str(), param.value.string_value.c_str());
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
            RCLCPP_ERROR(node_->get_logger(), "Error getting future result for behavior_server");
            return BT::NodeStatus::FAILURE;
        }
        last_decision_mode_= decision_mode_;
        return BT::NodeStatus::SUCCESS;
    }

} // namespace nav2_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::ChangeModeAction>("ChangeMode");
}