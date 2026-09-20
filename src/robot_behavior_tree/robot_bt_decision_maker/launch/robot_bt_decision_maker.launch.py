from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

yaml_path = os.path.join(
    get_package_share_directory('robot_bring_up'),
    'config',
    'sentry.yaml'
)

params_file = LaunchConfiguration(
    "params_file",
    default=yaml_path,
)

bt_xml_filename = LaunchConfiguration(
    'bt_xml_filename',
    default="/behavior_trees/RMUC_test/testt.xml"  
)

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=yaml_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    
    declare_bt_xml_filename_cmd = DeclareLaunchArgument(
        'bt_xml_filename',
        default_value="/behavior_trees/RMUC_test/template_test.xml",# 添加默认值# checklist
        description='Full path to the xml file to use for bt'
    )
    node_01 = Node(
        package="robot_bt_decision_maker",
        executable="robot_bt_decision_maker_node",
        output="screen",
        name="robot_bt_decision_maker_node",
        parameters=[
            params_file,  # 使用params_file的值作为参数文件路径
            {'bt_xml_filename': bt_xml_filename}  # 正确使用bt_xml_filename的值
        ]
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [
            declare_params_file_cmd,
            declare_bt_xml_filename_cmd, 
            node_01
        ]
    )
    # 返回让ROS2根据launch描述执行节点
    return launch_description