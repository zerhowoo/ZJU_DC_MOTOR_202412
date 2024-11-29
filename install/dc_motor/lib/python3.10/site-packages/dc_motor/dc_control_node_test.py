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
        self.publisher = self.create_publisher(JointTrajectory, '/trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(0.04, self.publish_trajectory)     # 定时器频率 20 Hz 0.05 s，10-50 Hz 保证实时性
        self.current_positions = [0.0]                                   # 初始化当前位置
        self.get_logger().info("DCControlNode started: sending cyclic trajectory.")

    def listener_callback(self, msg): # 打印收到的关节状态 
        self.get_logger().info(f"Received joint state: {msg.position}")   

    def publish_trajectory(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ["joint_0"]    # ["joint_0", "joint_1", "joint_2", "joint_3"]

        # 定义轨迹点 
        point1 = JointTrajectoryPoint(
            positions=[0.0], 
            time_from_start=rclpy.duration.Duration(seconds=1).to_msg()
            ) 
        point2 = JointTrajectoryPoint(
            positions=[200000.0], 
            time_from_start=rclpy.duration.Duration(seconds=10).to_msg()
            ) 
        point3 = JointTrajectoryPoint(
            positions=[200000.0], 
            time_from_start=rclpy.duration.Duration(seconds=15).to_msg()
            ) 
        point4 = JointTrajectoryPoint(
            positions=[0.0], 
            time_from_start=rclpy.duration.Duration(seconds=25).to_msg()
            )

        msg.points = [point1, point2, point3, point4]

        self.publisher.publish(msg)
        self.get_logger().info('Published JointTrajectory message for cyclic motion.')

def main(args=None):
    rclpy.init(args=args)
    dc_control_node_test = DCControlNode()
    rclpy.spin(dc_control_node_test)
    dc_control_node_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

