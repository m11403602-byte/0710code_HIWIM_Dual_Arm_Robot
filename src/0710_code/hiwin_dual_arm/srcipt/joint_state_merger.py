# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState

# class JointStateMerger(Node):
#     def __init__(self):
#         super().__init__('joint_state_merger')
#         # 👇 修正監聽的 Topic 名稱，對齊 ROS 2 預設的 Namespace 廣播路徑
#         self.sub_small = self.create_subscription(JointState, '/small_arm/joint_states', self.small_cb, 10)
#         self.sub_big = self.create_subscription(JointState, '/big_arm/joint_states', self.big_cb, 10)
        
#         # 發佈給 MoveIt 聽的統一全域頻道
#         self.pub = self.create_publisher(JointState, '/joint_states', 10)
        
#         self.small_state = JointState()
#         self.big_state = JointState()
#         self.create_timer(0.02, self.timer_cb) # 50Hz 合併發佈頻率

#     def small_cb(self, msg):
#         self.small_state = msg

#     def big_cb(self, msg):
#         self.big_state = msg

#     def timer_cb(self):
#         if not self.small_state.name or not self.big_state.name:
#             return
        
#         merged = JointState()
#         merged.header.stamp = self.get_clock().now().to_msg()
#         # 將小手臂和大手臂的陣列接起來
#         merged.name = self.small_state.name + self.big_state.name
#         merged.position = list(self.small_state.position) + list(self.big_state.position)
        
#         if self.small_state.velocity and self.big_state.velocity:
#             merged.velocity = list(self.small_state.velocity) + list(self.big_state.velocity)
            
#         self.pub.publish(merged)

# def main():
#     rclpy.init()
#     node = JointStateMerger()
#     print("🔄 網路模擬情報官已啟動：正在跨網段合併 B 與 C 的狀態...")
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
# 做法 B：用主機時鐘重蓋 /joint_states 時戳（過 MoveIt 1 秒新鮮度門檻），
# 訂 /big_arm,/small_arm/joint_states 後拼成 12 軸 /joint_states。無 liveness guard。
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateMerger(Node):
    def __init__(self):
        super().__init__('joint_state_merger')
        self.sub_small = self.create_subscription(JointState, '/small_arm/joint_states', self.small_cb, 10)
        self.sub_big = self.create_subscription(JointState, '/big_arm/joint_states', self.big_cb, 10)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.small_state = JointState()
        self.big_state = JointState()
        self.create_timer(0.02, self.timer_cb)  # 50 Hz

    def small_cb(self, msg):
        self.small_state = msg

    def big_cb(self, msg):
        self.big_state = msg

    def timer_cb(self):
        if not self.small_state.name or not self.big_state.name:
            return
        merged = JointState()
        merged.header.stamp = self.get_clock().now().to_msg()  # ★ 重蓋成主機時鐘
        merged.name = list(self.small_state.name) + list(self.big_state.name)
        merged.position = list(self.small_state.position) + list(self.big_state.position)
        if self.small_state.velocity and self.big_state.velocity:
            merged.velocity = list(self.small_state.velocity) + list(self.big_state.velocity)
        self.pub.publish(merged)


def main():
    rclpy.init()
    rclpy.spin(JointStateMerger())
    rclpy.shutdown()


if __name__ == '__main__':
    main()