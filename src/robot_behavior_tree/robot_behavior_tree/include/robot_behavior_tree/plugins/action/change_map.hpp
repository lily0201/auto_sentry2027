//
// Created by elsa on 24-12-28.
//

#ifndef SWITCH_MAP_HPP
#define SWITCH_MAP_HPP

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "yaml-cpp/yaml.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

    /**
     * @brief 切换map节点，读取新map的yaml文件和图片文件，转换为栅格地图nav_msgs::msg::OccupancyGrid形式，发布到map话题上
     * @brief 由于nav2的map_server节点只在运行一开始时发布一次map，因此map可以被新的map覆盖
     * @brief 需要注意的是，如果在原有栅格地图中占有率为100的格子在新地图中占有率为0,全局地图中这部分的障碍物层和膨胀层消失有一定的延迟
     */
    class ChangeMapAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        ChangeMapAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        ChangeMapAction() = delete;

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
              BT::InputPort<std::string>("map_yaml", "map_yaml_name"), //新map的yaml文件
              BT::InputPort<std::string>("map_image", "map_image_name"), //新map的image文件
            };
        }

    private:
        /**
         * @brief 发布新地图函数
         * @param yaml_file 新map的yaml文件路径
         * @param map_file 新map的image文件路径
         */
        void publish_map(const std::string &yaml_file, const std::string &map_file);

        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
        ///新map的yaml文件
        std::string map_yaml_;
        ///新map的image文件
        std::string map_image_;
    };

} // namespace nav2_behavior_tree

#endif //SWITCH_MAP_HPP
