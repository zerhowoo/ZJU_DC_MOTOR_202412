import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class DCControlNode(Node):
    def __init__(self):
        super().__init__('dc_control')

        # 发布和订阅
        self.publisher = self.create_publisher(JointTrajectory, '/trajectory_controller/joint_trajectory', 10)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.listener_callback, 10)
        # 定时器
        self.timer = self.create_timer(0.1, self.timer_callback)
        # 状态管理
        self.is_motion_complete = True  # 运动是否完成
        self.current_position = 0.0     # 当前电机位置
        self.target_position = 0.0      # 目标电机位置

        # 目标轨迹配置
        self.trajectory_points = [
            (0.0, 2.0),    # (位置, 相对时间)
            (400000.0, 12.0),
            (400000.0, 15.0),
            (0.0, 25.0),
        ]
        self.trajectory_index = 0  # 当前轨迹循环索引

    def listener_callback(self, msg):
        """更新当前电机位置"""
        self.current_position = msg.position[0]  # 假设有一个关节电机
        self.get_logger().info(f"当前位置: {self.current_position:.2f}")

        # 判断运动是否完成
        if abs(self.current_position - self.target_position) < 1:  # 允许微小误差
            self.is_motion_complete = True

    def publish_trajectory(self):
        """发布轨迹消息"""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['joint_0']              # 电机名称需要与URDF定义一致

        # 构建轨迹点
        for pos, time in self.trajectory_points:
            point = JointTrajectoryPoint()
            point.positions = [pos]
            point.time_from_start = rclpy.duration.Duration(seconds=time).to_msg()
            msg.points.append(point)

        self.publisher.publish(msg)
        self.get_logger().info(f"发布轨迹: 目标位置 {[p[0] for p in self.trajectory_points]}")

    def timer_callback(self):
        """定时器回调：控制轨迹发布"""
        if self.is_motion_complete:
            self.is_motion_complete = False       # 标志位重置，表示轨迹执行中
            self.target_position = self.trajectory_points[-1][0]  # 更新目标位置（最后一个点）
            self.publish_trajectory()

def main(args=None):
    rclpy.init(args=args)
    node = DCControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
