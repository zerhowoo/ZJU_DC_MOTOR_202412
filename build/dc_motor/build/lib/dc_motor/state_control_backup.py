import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import logging

# 配置日志记录
logging.basicConfig(filename='output.txt', level=logging.INFO, format='%(asctime)s - %(message)s') 

class DCControlNode(Node):
    def __init__(self):
        super().__init__('dc_control')

        # 发布和订阅
        self.publisher = self.create_publisher(JointTrajectory, '/trajectory_controller/joint_trajectory', 10)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.listener_callback, 10)
        
        # 定时器
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 状态管理 
        self.current_state = "WAIT"           # 初始状态为等待 
        self.current_position = 0.0           # 当前位置
        self.target_position = 0.0            # 本轮目标位置
        self.pre_tar_position = None          # 上轮目标位置 
        self.cycle_completed = True           # 是否完成当前轨迹

        # 目标轨迹配置
        self.trajectory_points = [
            (200000.0, 10.0),    # 
            (200000.0, 5.0),     # 
            (400000.0, 10.0),    # 
            (400000.0, 10.0),    #
            (0.0, 10.0),         # 
        ]
        self.trajectory_index = 0  # 当前轨迹点索引

    def listener_callback(self, msg):
        """订阅回调函数，更新当前电机位置"""
        self.current_position = msg.position[0]  # 假设有一个关节电机
        self.get_logger().info(f"当前位置: {self.current_position:.2f}")
        logging.info(f"当前位置: {self.current_position:.2f}")

        # 判断运动是否完成
        if abs(self.current_position - self.target_position) < 1e-1:  # 允许微小误差
            if self.current_state == "MOVING": 
                self.get_logger().info("运动已完成.") 
                logging.info("运动已完成.") 
                self.cycle_completed = True 
                self.current_state = "WAIT"

    def publish_trajectory(self):
        """发布当前周期的目标轨迹""" 
        next_position, action_time = self.trajectory_points[self.trajectory_index] 
        
        # 判断是否需要发布新轨迹 
        if self.pre_tar_position == next_position: 
            self.get_logger().info(f"保持静止在位置: {next_position}，持续时间: {action_time}s") # 改动处：打印静止信息
            logging.info(f"保持静止在位置: {next_position}，持续时间: {action_time}s")  
            self.pre_tar_position = self.target_position                                     # 更新上一轮目标位置 
            self.target_position = next_position                                             # 保持当前目标位置

            # 直接更新索引，跳过发布 
            self.trajectory_index = (self.trajectory_index + 1) % len(self.trajectory_points) 
            logging.info(f"trajectory_index 等于: {self.trajectory_index}") 
            self.cycle_completed = True # 标记完成当前目标 
            return
        
        """发布轨迹消息"""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['joint_0']              # 电机名称需要与URDF定义一致
        point = JointTrajectoryPoint()
        
        point.positions = [next_position] 
        point.time_from_start = rclpy.duration.Duration(seconds=action_time).to_msg() 
        msg.points.append(point) 
        
        self.pre_tar_position = self.target_position # 更新上一目标位置 
        self.target_position = next_position         # 更新当前目标位置 
        self.publisher.publish(msg) 
        self.get_logger().info(f"Published trajectory point: {next_position} at time {action_time:.1f}s") # 更新轨迹索引 
        logging.info(f"Published trajectory point: {next_position} at time {action_time:.1f}s")           
        
        # 更新轨迹索引 
        self.trajectory_index = (self.trajectory_index + 1) % len(self.trajectory_points) 
        self.current_state = "MOVING"

    def timer_callback(self):
        """周期性状态检查，并决定是否发布新轨迹"""
        if self.cycle_completed:
            self.cycle_completed = False       # 标志位重置，表示轨迹执行中
            self.publish_trajectory()

def main(args=None):
    rclpy.init(args=args)
    node = DCControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
