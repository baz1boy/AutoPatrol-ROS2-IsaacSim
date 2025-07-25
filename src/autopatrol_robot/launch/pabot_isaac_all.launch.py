import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

 # 获取每个功能包的路径
    pabot_navigation2_pkg_dir = get_package_share_directory('patrol_robot_navigation2')
    autopatrol_pkg_dir = get_package_share_directory('autopatrol_robot')

    # 构建 launch 文件路径
    nav2_launch = os.path.join(pabot_navigation2_pkg_dir, 'launch', 'isaac_navigation2.launch.py')
    autopatrol_launch = os.path.join(autopatrol_pkg_dir, 'launch', 'autopatrol.launch.py')

    # 包含动作1: 启动 Gazebo 仿真和机器人


    # 包含动作2: 启动 Nav2 导航系统
    launch_navigation2_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'use_sim_time': 'true',
            'map': os.path.join(pabot_navigation2_pkg_dir, 'maps', 'pabot_room_navigation.yaml'),
            'params_file': os.path.join(pabot_navigation2_pkg_dir, 'config', 'nav2_params.yaml'),
        }.items()
    )

    # 包含动作3: 启动巡逻和语音节点
    launch_patrol_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(autopatrol_launch)
    )

    return LaunchDescription([
        
        launch_navigation2_action,
        TimerAction(
            period=3.0,  # 等待 5 秒
            actions=[launch_patrol_action]
        )
    ])

