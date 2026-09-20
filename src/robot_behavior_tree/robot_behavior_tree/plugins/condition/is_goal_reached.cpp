#include <string>

#include "robot_behavior_tree/plugins/condition/is_goal_reached.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace nav2_behavior_tree
{
    IsGoalReachedCondition::IsGoalReachedCondition(
        const std::string& condition_name,
        const BT::NodeConfiguration& conf)
        : BT::ConditionNode(condition_name, conf),
          global_frame_("map"),
          robot_base_frame_("base_link"),
          is_goal_reached(false)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        aim_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot/aim_pose",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&IsGoalReachedCondition::AimPoseCallback, this, std::placeholders::_1),
            sub_option);

        tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
        config().blackboard->set<bool>("is_goal_reached", is_goal_reached);
        if (!node_->has_parameter("transform_tolerance"))
        {
            node_->declare_parameter("transform_tolerance", 0.15);
        }
        node_->get_parameter("transform_tolerance", transform_tolerance_);

        config().blackboard->get<bool>("is_in_chase", is_in_chase);
    }

    BT::NodeStatus IsGoalReachedCondition::tick()
    {
        config().blackboard->get<bool>("is_in_chase", is_in_chase);
        if (is_in_chase) { //在追击则使用裁剪路径后的目标点
            callback_group_executor_.spin_some();
            current_goal.pose.position.x = current_aim_pose_.pose.position.x;
            current_goal.pose.position.y = current_aim_pose_.pose.position.y;
            RCLCPP_INFO(node_->get_logger(), "is_goal_reached: 使用追击目标点");
        }
        else {
            getInput("goal", current_goal);
            RCLCPP_INFO(node_->get_logger(), "is_goal_reached: 使用普通目标点");
        }

        geometry_msgs::msg::PoseStamped current_pose;
        if (!nav2_util::getCurrentPose(
            current_pose, *tf_, global_frame_, robot_base_frame_,
            transform_tolerance_))
        {
            RCLCPP_INFO(node_->get_logger(), "is_goal_reached:没有获取到TF变换");
            return BT::NodeStatus::FAILURE;
        }
        config().blackboard->set<double>("robot_pos_x", current_pose.pose.position.x);
        config().blackboard->set<double>("robot_pos_y", current_pose.pose.position.y);
        if (fabs(current_goal.pose.position.x - current_pose.pose.position.x) <= 0.2 &&
            fabs(current_goal.pose.position.y - current_pose.pose.position.y) <= 0.2)
        {
            is_goal_reached = true;
            config().blackboard->set<bool>("is_goal_reached", is_goal_reached);
            RCLCPP_INFO(node_->get_logger(), "当前机器人处于目标点 x: %lf, y: %lf",
                        current_goal.pose.position.x, current_goal.pose.position.y);
            return BT::NodeStatus::SUCCESS;
        }
        if (current_goal.pose.position.x==0&&current_goal.pose.position.y==0)
        {
            RCLCPP_INFO(node_->get_logger(), "is_goal_reached:当前机器人无目标点 ");
            return BT::NodeStatus::SUCCESS;
        }
        is_goal_reached = false;
        config().blackboard->set<bool>("is_goal_reached", is_goal_reached);
        RCLCPP_INFO(node_->get_logger(), "当前机器人未处于目标点 x: %lf, y: %lf",
                    current_goal.pose.position.x, current_goal.pose.position.y);
        return BT::NodeStatus::FAILURE;
    }

    void IsGoalReachedCondition::AimPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_aim_pose_.pose.position.x = msg->pose.position.x;
        current_aim_pose_.pose.position.y = msg->pose.position.y;
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IsGoalReachedCondition>("IsGoalReached");
}
