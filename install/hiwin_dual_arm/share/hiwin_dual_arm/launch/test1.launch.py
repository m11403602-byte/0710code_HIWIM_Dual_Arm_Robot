from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 讀取雙臂設定包
    moveit_config = MoveItConfigsBuilder("dual_hiwin", package_name="hiwin_dual_arm").to_moveit_configs()
    launch_package_path = moveit_config.package_path

    ld = LaunchDescription()
    
    # 參數宣告
    ld.add_action(DeclareLaunchArgument("use_rviz", default_value="true"))

    # 1. 廣播虛擬關節 (Virtual Joints)
    virtual_joints_launch = launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # 2. 啟動 Robot State Publisher (負責計算並廣播 12 軸 TF 樹)
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/rsp.launch.py")
            ),
        )
    )

    # 3. 啟動 MoveGroup (MoveIt 核心大腦，負責算避障軌跡)
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # 4. 啟動 RViz2 (視覺化操作介面)
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )
    return ld