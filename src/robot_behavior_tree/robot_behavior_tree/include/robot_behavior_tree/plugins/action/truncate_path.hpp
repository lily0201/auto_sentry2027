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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRUNCATE_PATH_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRUNCATE_PATH_ACTION_HPP_

#include <string>
#include <memory>
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "rm_interfaces/msg/truncate_path_info.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{
    /**
     * @brief A BT::ActionNodeBase to shorten path by some distance
     */
    class TruncatePath : public BT::ActionNodeBase
    {
    public:
        /**
         * @brief A nav2_behavior_tree::TruncatePath constructor
         * @param xml_tag_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        TruncatePath(
            const std::string& xml_tag_name,
            const BT::NodeConfiguration& conf);

        /**
         * @brief Creates list of BT ports
         * @return BT::PortsList Containing basic ports along with node-specific ports
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<nav_msgs::msg::Path>("input_path", "Original Path"),
                BT::OutputPort<nav_msgs::msg::Path>("output_path", "Path truncated to a certain distance"),
            };
        }

    private:
        /**
         * @brief The other (optional) override required by a BT action.
         */
        void halt() override
        {
        }

        /**
         * @brief The main override required by a BT action
         * @return BT::NodeStatus Status of tick execution
         */
        BT::NodeStatus tick() override;

        /**
         * @brief 接收裁剪路径的相关信息
         * @param msg
         */
        void TruncatePathInfoCallback(rm_interfaces::msg::TruncatePathInfo::SharedPtr msg);

        /// 裁剪路径长度
        double distance_;
        /// 是否在打符模式
        bool if_dafu;

        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        /// 接收裁剪路径信息
        rclcpp::Subscription<rm_interfaces::msg::TruncatePathInfo>::SharedPtr truncate_path_info_sub_;
        /// 发布裁剪路径后最终目标点给决策，用于判断goal reach
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr aim_pose_pub_;

        geometry_msgs::msg::PoseStamped aim_pose_;
        rm_interfaces::msg::TruncatePathInfo truncate_path_info_;
    };
} // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRUNCATE_PATH_ACTION_HPP_
