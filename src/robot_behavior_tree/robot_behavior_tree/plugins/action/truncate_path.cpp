// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "behaviortree_cpp_v3/decorator_node.h"
#include "robot_behavior_tree/plugins/action/truncate_path.hpp"

namespace nav2_behavior_tree
{
    TruncatePath::TruncatePath(
        const std::string& name,
        const BT::NodeConfiguration& conf)
        : BT::ActionNodeBase(name, conf),
          distance_(0.0)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        truncate_path_info_sub_ = node_->create_subscription<rm_interfaces::msg::TruncatePathInfo>(
            "/robot/truncated_path_info",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&TruncatePath::TruncatePathInfoCallback, this, std::placeholders::_1),
            sub_option);
        aim_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/robot/aim_pose",
            rclcpp::SystemDefaultsQoS());
    }

    inline BT::NodeStatus TruncatePath::tick()
    {
        setStatus(BT::NodeStatus::RUNNING);

        callback_group_executor_.spin_some();
        RCLCPP_INFO(node_->get_logger(), "^^^^^^^^^^^^^distance to truncate: %lf^^^^^^^^^^^^", distance_);

        nav_msgs::msg::Path input_path;

        getInput("input_path", input_path); //获取裁剪路径前path

        if (input_path.poses.empty())
        {
            setOutput("output_path", input_path);
            return BT::NodeStatus::SUCCESS;
        }

        geometry_msgs::msg::PoseStamped final_pose = input_path.poses.back();
        RCLCPP_INFO(node_->get_logger(), "裁剪前final_pose: %lf, %lf, %lf, %lf",
            input_path.poses.back().pose.orientation.x, input_path.poses.back().pose.orientation.y,
            input_path.poses.back().pose.orientation.z, input_path.poses.back().pose.orientation.w);

        //计算当前路径最后一个点到原目标点final_pose的距离
        double distance_to_goal = nav2_util::geometry_utils::euclidean_distance(
            input_path.poses.back(), final_pose);

        //裁剪至distance_to_goal < distance_为止
        while (distance_to_goal < distance_ && input_path.poses.size() > 2)
        {
            input_path.poses.pop_back();
            distance_to_goal = nav2_util::geometry_utils::euclidean_distance(
                input_path.poses.back(), final_pose);
            // RCLCPP_INFO(node_->get_logger(), "current distance to goal: %f", distance_to_goal);
        }
        RCLCPP_INFO(node_->get_logger(), "final distance to goal: %f", distance_to_goal);
        //若有进行裁剪路径，则将新的目标点发送到决策
        if (distance_ != 0) {
            aim_pose_ = input_path.poses.back();
            aim_pose_pub_->publish(aim_pose_);
            RCLCPP_INFO(node_->get_logger(), "----------current aim pose: %lf, %lf",
                        aim_pose_.pose.position.x, aim_pose_.pose.position.y);
        }

        double dx = final_pose.pose.position.x - input_path.poses.back().pose.position.x;
        double dy = final_pose.pose.position.y - input_path.poses.back().pose.position.y;

        double final_angle = atan2(dy, dx);

        if (std::isnan(final_angle) || std::isinf(final_angle))
        {
            RCLCPP_WARN(
                config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
                "Final angle is not valid while truncating path. Setting to 0.0");
            final_angle = 0.0;
        }

        if(if_dafu) //打符模式下控制特定转向
        {
            input_path.poses.back().pose.orientation.x = truncate_path_info_.final_pose.pose.orientation.x;
            input_path.poses.back().pose.orientation.y = truncate_path_info_.final_pose.pose.orientation.y;
            input_path.poses.back().pose.orientation.z = truncate_path_info_.final_pose.pose.orientation.z;
            input_path.poses.back().pose.orientation.w = truncate_path_info_.final_pose.pose.orientation.w;
        }
        else
        {
            input_path.poses.back().pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
                final_angle);
        }
        RCLCPP_INFO(node_->get_logger(), "input_path.poses.back().pose.orientation: %lf, %lf, %lf, %lf",
            input_path.poses.back().pose.orientation.x, input_path.poses.back().pose.orientation.y,
            input_path.poses.back().pose.orientation.z, input_path.poses.back().pose.orientation.w);

        setOutput("output_path", input_path);
        RCLCPP_INFO(node_->get_logger(), "---------Truncate path end--------");

        return BT::NodeStatus::SUCCESS;
    }

    void TruncatePath::TruncatePathInfoCallback(rm_interfaces::msg::TruncatePathInfo::SharedPtr msg)
    {
        distance_ = msg->distance;
        if_dafu = msg->dafu;
        truncate_path_info_.final_pose.pose.orientation.x = msg->final_pose.pose.orientation.x;
        truncate_path_info_.final_pose.pose.orientation.y = msg->final_pose.pose.orientation.y;
        truncate_path_info_.final_pose.pose.orientation.z = msg->final_pose.pose.orientation.z;
        truncate_path_info_.final_pose.pose.orientation.w = msg->final_pose.pose.orientation.w;
        RCLCPP_INFO(node_->get_logger(), "received final_pose.pose.orientation: %lf, %lf, %lf, %lf",
            truncate_path_info_.final_pose.pose.orientation.x, truncate_path_info_.final_pose.pose.orientation.y,
            truncate_path_info_.final_pose.pose.orientation.z, truncate_path_info_.final_pose.pose.orientation.w);
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::TruncatePath>("TruncatePath");
}
