from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_helloworld',  # 替换为你的包名
            executable='ros2_helloworld',     # 替换为你的可执行文件名
            name='helloworld',            # 节点名称
            output='screen',                # 输出到屏幕
            # parameters=[{'param_name': 'param_value'}]  # 可选参数
        )
    ])
