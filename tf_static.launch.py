from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. OT128 (지붕 위) 위치 정의
        # base_link -> ot128_frame
        # 위치: x=0m(중앙), y=0m(중앙), z=1.8m(높이)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='ot128_tf',
            arguments=['0', '0', '1.8', '0', '0', '0', 'base_link', 'ot128_frame']
        ),

        # 2. QT128 (전면 범퍼) 위치 정의
        # base_link -> qt128_frame
        # 위치: x=1.5m(앞으로), y=0m, z=0.5m(높이) / 약간 아래를 봄(pitch)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='qt128_tf',
            # arguments 순서: x y z yaw pitch roll parent child
            arguments=['1.5', '0', '0.5', '0', '0', '0', 'base_link', 'qt128_frame']
        )
    ])
