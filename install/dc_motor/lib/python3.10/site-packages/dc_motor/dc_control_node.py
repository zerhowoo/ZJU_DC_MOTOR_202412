import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class DCControlNode(Node):
    def __init__(self):
        super().__init__('dc_control')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10
        )
        # 发布 JointTrajectory 消息
        self.publisher = self.create_publisher(JointTrajectory, '/trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(1.0, self.publish_trajectory)     # 定时器频率 20 Hz 0.05 s，10-50 Hz 保证实时性
        self.current_positions = [0.0]                   # 初始化当前位置

        # 设置处理消息的频率,应该略小于 定时器频率
        self.process_frequency = 0.04                  # 5.0 Hz 
        self.last_processed_time = self.get_clock().now()

    def listener_callback(self, msg):
        current_time = self.get_clock().now()
        time_diff = current_time - self.last_processed_time

        # 仅在特定频率下处理消息
        if time_diff.nanoseconds / 1e9 >= 1.0 / self.process_frequency:
            self.current_positions = msg.position        # 更新当前位置
            self.get_logger().info(f'Updated current positions: {self.current_positions}')
            self.last_processed_time = current_time

    def publish_trajectory(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ["joint_0"]    #["joint_0", "joint_1", "joint_2", "joint_3"]

        point1 = JointTrajectoryPoint()
        point1.positions = self.current_positions  # 使用当前位置作为第一个点的位置
        point1.velocities = [0.0]        # [0.0, 0.0, 0.0, 0.0]
        point1.accelerations = [0.0]     # [0.0, 0.0, 0.0, 0.0]
        point1.time_from_start.sec = 1   # 等1 秒再动
        point1.time_from_start.nanosec = 0

        point2 = JointTrajectoryPoint()
        point2.positions = [500000.0]    # [500000.0, 500000.0, 500000.0, 500000.0]
        point2.velocities = [0.0]        # [0.0, 0.0, 0.0, 0.0]
        point2.accelerations = [0.0]     # [0.0, 0.0, 0.0, 0.0]
        point2.time_from_start.sec = 11  # 经过10 秒到达 500000位置
        point2.time_from_start.nanosec = 0

        point3 = JointTrajectoryPoint()
        point3.positions = [500000.0]    # [500000.0, 500000.0, 500000.0, 500000.0]
        point3.velocities = [0.0]        # [0.0, 0.0, 0.0, 0.0]
        point3.accelerations = [0.0]     # [0.0, 0.0, 0.0, 0.0]
        point3.time_from_start.sec = 15  # 500000的 位置静止四秒
        point3.time_from_start.nanosec = 0

        point4 = JointTrajectoryPoint()
        point4.positions = [0.0]         # [0.0, 0.0, 0.0, 0.0]
        point4.velocities = [0.0]        # [0.0, 0.0, 0.0, 0.0]
        point4.accelerations = [0.0]     # [0.0, 0.0, 0.0, 0.0]
        point4.time_from_start.sec = 25  # 经过10 秒到达 0位置
        point4.time_from_start.nanosec = 0

        msg.points = [point1, point2, point3, point4]

        self.publisher.publish(msg)
        self.get_logger().info('Published JointTrajectory message')

def main(args=None):
    rclpy.init(args=args)
    dc_control_node = DCControlNode()
    rclpy.spin(dc_control_node)
    dc_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

