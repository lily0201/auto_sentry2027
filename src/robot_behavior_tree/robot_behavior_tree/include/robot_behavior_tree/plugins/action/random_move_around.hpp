#ifndef RANDOM_MOVE_AROUND_HPP_
#define RANDOM_MOVE_AROUND_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "behaviortree_cpp_v3/action_node.h"

#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"

#include "nav2_util/robot_utils.hpp"

#include <random> // 包含随机数库

namespace nav2_behavior_tree
{

    /**
     * @brief 随机移动节点，每次tick会在指定的四个点中随机选取一个目标点进行导航
     */
    class RandomMoveAroundAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        RandomMoveAroundAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        RandomMoveAroundAction() = delete;

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
            return
            {
                BT::InputPort<float>("select_time", "select_time"),
                BT::InputPort<double>("moving_distance", "moving distance"),
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "goal"),
            };
        }

    private:
        rclcpp::Node::SharedPtr node_;

        ///两次选取目标点之间的时间间隔
        float select_time;
        ///计数阈值，达到阈值表示已经经过相应的时间
        int count_size_ ;
        std::chrono::milliseconds bt_loop_duration_;

        ///指定点x
        double robot_position_x;
        ///指定点y
        double robot_position_y;
        geometry_msgs::msg::PoseStamped pose;
        ///运动范围
        double moving_distance;

        std::string global_frame_;
        std::string robot_base_frame_;
        double transform_tolerance_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
    };  

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
