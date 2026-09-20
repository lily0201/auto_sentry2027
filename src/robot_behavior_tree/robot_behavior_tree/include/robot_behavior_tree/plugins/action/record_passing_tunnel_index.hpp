//
// Created by elsa on 25-5-15.
//

#ifndef RECORD_PASSING_TUNNEL_INDEX_HPP
#define RECORD_PASSING_TUNNEL_INDEX_HPP

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "rm_interfaces/msg/lidarposition.hpp"
#include "rm_interfaces/msg/lidarstation.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief 记录导航途径路上需要通过的狗洞序号
     */
    class RecordPassingTunnelIndexAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        RecordPassingTunnelIndexAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        RecordPassingTunnelIndexAction() = delete;

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
        struct Point
        {
            double x;
            double y;
        };

        bool IsPointInPolygon(const Point& vtPoint, const std::vector<Point>& vecPoints);
        void lidarstationCallback(const rm_interfaces::msg::Lidarstation::ConstPtr &msg);

        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<rm_interfaces::msg::Lidarstation>::SharedPtr lidarstation_sub_;
        rm_interfaces::msg::Lidarstation::SharedPtr lidarstation_;

        ///红方前哨狗洞四个范围端点 x
        std::vector<double> red_outpost_tunnel_x_;
        ///红方前哨狗洞四个范围端点 y
        std::vector<double> red_outpost_tunnel_y_;
        ///红方沟槽（香蕉道）六个范围端点 x
        std::vector<double> red_banana_tunnel_x_;
        ///红方沟槽（香蕉道）六个范围端点 y
        std::vector<double> red_banana_tunnel_y_;
        ///蓝方前哨狗洞四个范围端点 x
        std::vector<double> blue_outpost_tunnel_x_;
        ///蓝方前哨狗洞四个范围端点 y
        std::vector<double> blue_outpost_tunnel_y_;
        ///蓝方沟槽（香蕉道）六个范围端点 x
        std::vector<double> blue_banana_tunnel_x_;
        ///蓝方沟槽（香蕉道）六个范围端点 y
        std::vector<double> blue_banana_tunnel_y_;

        std::vector<Point> red_outpost_tunnel;
        std::vector<Point> red_banana_tunnel;
        std::vector<Point> blue_outpost_tunnel;
        std::vector<Point> blue_banana_tunnel;

        ///路径上经过的狗洞编号，若一条路径上同时经过多个狗洞，则取第一个经过的狗洞
        ///以红方半场前哨狗洞为序号1, 2 -- 红方沟槽狗洞，3 -- 蓝方前哨狗洞，4 -- 蓝方沟槽狗洞，0 -- 当前不经过狗洞
        int tunnel_index_;
        ///当前是否在狗洞中
        bool if_in_tunnel_;
        bool if_last_in_tunnel_;
        ///狗洞是否有人堵
        std::vector<bool> dangerous_tunnel;

        std::shared_ptr<tf2_ros::Buffer> tf_;
        std::string global_frame_;
        std::string robot_base_frame_;
        ///导航控制模式
        int mode;
        double transform_tolerance_;

        bool if_in_mode_changing;
    };

} // namespace nav2_behavior_tree

#endif //RECORD_PASSING_TUNNEL_INDEX_HPP
