#include <string>

#include "robot_behavior_tree/plugins/condition/if_in_ramp.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace nav2_behavior_tree
{

    IfInRampCondition::IfInRampCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
        global_frame_("map"),
        robot_base_frame_("base_link"),
        is_in_ramp(false)
    {
        getInput("global_frame", global_frame_);
        getInput("robot_base_frame", robot_base_frame_);
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
        config().blackboard->set<bool>("is_in_ramp",is_in_ramp);
        if (!node_->has_parameter("transform_tolerance"))
        {
            node_->declare_parameter("transform_tolerance", 0.15);
        }
        node_->get_parameter("transform_tolerance", transform_tolerance_);
    }

    BT::NodeStatus IfInRampCondition::tick()
    {
        geometry_msgs::msg::PoseStamped current_pose;
        if (!nav2_util::getCurrentPose(
                current_pose, *tf_, global_frame_, robot_base_frame_,
                transform_tolerance_))
        {
            RCLCPP_INFO(node_->get_logger(), "没有获取到TF变换");
            return BT::NodeStatus::FAILURE;
        }
        tf2::Quaternion quaternion(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
        tf2::Matrix3x3 euler(quaternion);
        double roll, pitch, yaw;
        euler.getRPY(roll, pitch, yaw);
        if((fabs(roll)<transform_tolerance_)&&(fabs(pitch)<transform_tolerance_))
        {
            is_in_ramp = false;
            config().blackboard->set<bool>("is_in_ramp",is_in_ramp);
            RCLCPP_INFO(node_->get_logger(), "当前机器人处于平地 roll: %lf, pitch: %lf", roll, pitch);
            return BT::NodeStatus::FAILURE;
        }
        else{
            is_in_ramp = true;
            config().blackboard->set<bool>("is_in_ramp",is_in_ramp);
            RCLCPP_INFO(node_->get_logger(), "当前机器人处于平地坡道 roll: %lf, pitch: %lf", roll, pitch);
            return BT::NodeStatus::SUCCESS;
        }  
    }
    double IfInRampCondition::formatAngle(double angle)
    {
        while (angle > M_PI)
        {
            angle -= 2 * M_PI;
        }
        while (angle < -M_PI)
        {
            angle += 2 * M_PI;
        }
        return angle / M_PI * 180;
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IfInRampCondition>("IfInRamp");
}
