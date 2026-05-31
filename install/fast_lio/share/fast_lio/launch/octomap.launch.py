import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'resolution': 0.05,
                'frame_id': 'camera_init',     # fast_lio的全局坐标系
                'base_frame_id': 'body',       # 雷达的局部坐标系
                'sensor_model.max_range': 30.0,
                'occupancy_min_z': -1.0,       # 先放开限制，看看2D图是否出现黑色的障碍物
                'occupancy_max_z': 3.0,
                'pointcloud_min_z': -1.0,
                'pointcloud_max_z': 3.0,
                'latch': True,                 # 必须为True，否则RViz里接收不到
                'incremental_2D_projection': True, # 开启2D增量地图投影
                'publish_free_space': False,   # 关闭发布探索过的空闲区域（白色）
            }],
            remappings=[
                ('cloud_in', '/cloud_registered') # 订阅fast-lio输出的全局点云
            ]
        )
    ])
