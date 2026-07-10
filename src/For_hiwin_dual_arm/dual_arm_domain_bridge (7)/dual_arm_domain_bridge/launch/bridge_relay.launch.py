# Plan B 整層橋一鍵啟動: 端點透明轉送
# 內容 = 上行橋(必跑) + 端點透明轉送(下行)
#
# 用法:
#   ros2 launch dual_arm_domain_bridge bridge_relay.launch.py
#   ros2 launch dual_arm_domain_bridge bridge_relay.launch.py host_domain:=10 arm_a_domain:=20 arm_b_domain:=30
#
# 注意:
#   - 兩支程式內部用 set_domain_id() 明碼指定 domain（CLI 位置參數）,
#     「不吃」這個終端的 ROS_DOMAIN_ID 環境變數 — 改編號只改這裡的參數即可

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host_domain = LaunchConfiguration("host_domain")
    arm_a_domain = LaunchConfiguration("arm_a_domain")
    arm_b_domain = LaunchConfiguration("arm_b_domain")

    return LaunchDescription([
        DeclareLaunchArgument(
            "host_domain", default_value="10",
            description="主機(brain/move_group/RViz)所在 domain"),
        DeclareLaunchArgument(
            "arm_a_domain", default_value="20",
            description="A 臂(RA610-1476, big_)原廠系統所在 domain"),
        DeclareLaunchArgument(
            "arm_b_domain", default_value="30",
            description="B 臂(RA605-710, small_)原廠系統所在 domain"),

        # 上行橋: D_armA/D_armB 的 /joint_states → 加前綴合併 → D_host /joint_states
        Node(
            package="dual_arm_domain_bridge",
            executable="joint_state_uplink_bridge",
            name="joint_state_uplink_bridge",
            output="screen",
            arguments=[host_domain, arm_a_domain, arm_b_domain],
        ),

        # 下行(方案 C): 五端點透明轉送 — RViz 原生 Execute/Stop 可用, 失敗語意=直連
        Node(
            package="dual_arm_domain_bridge",
            executable="trajectory_downlink_endpoint_relay",
            name="trajectory_downlink_endpoint_relay",
            output="screen",
            arguments=[host_domain, arm_a_domain, arm_b_domain],
        ),
    ])
