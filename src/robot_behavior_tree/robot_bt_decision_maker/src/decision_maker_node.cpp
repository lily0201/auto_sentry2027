//
// Created by elsa on 25-1-31.
//

#include "robot_bt_decision_maker/robot_bt_decision_maker.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<DecisionMakerNode>("decision_maker_node");
    /* 运行节点，并检测退出信号*/
    // 当节点没有退出时，循环调用runBehaviorTree
    rclcpp::WallRate loop_rate(100);

    while (rclcpp::ok())
    {
        node->runBehaviorTree();
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}