# modified by Wang,Zihao at 2024.12.10 in Institute of Advanced Machines Zhejiang University

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from datetime import datetime
import time         
import logging     # 打印运动信息到文本文件中
import os          # 从文本文件中获取轨迹点信息 

# fetch current date and time，normalized as a part of dataname
current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S") 
log_filename = f'log_trajectory_book/output_{current_time}.txt' 

# config. log 
logging.basicConfig( filename=log_filename, level=logging.INFO, format='%(asctime)s - %(message)s' ) 

# set max. velocity
MAX_VELOCITY = 200000

def load_trajectory_points(file_path):
    """从文件加载轨迹点"""
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"文件 {file_path} 不存在！请创建并提供有效的轨迹点文件。")
    
    trajectory_points = []
    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()
            if line:
                try:
                    position1, position2, start_time, duration = map(float, line.split(','))   # 增加从站
                    trajectory_points.append(([position1, position2], start_time, duration))   # 增加从站
                except ValueError:
                    raise ValueError(f"文件格式错误：{line} 需要是 'position1, position2, start_time, duration' 格式。") # 增加从站
    return trajectory_points

class DCControlNode(Node):
    def __init__(self, trajectory_points):
        super().__init__('dc_control')

        # Definition of publisher and subscription
        self.publisher = self.create_publisher(JointTrajectory, '/trajectory_controller/joint_trajectory', 10)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.listener_callback, 10)
        
        # timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # managment of state maschine
        self.current_state = "WAIT"                   # 初始状态为等待 
        self.current_position = [0.0, 0.0]            # 当前两个电机的位置
        self.target_position = [0.0, 0.0]             # 当前两个电机的目标位置
        self.previous_target_position = [None, None]  # 上一个目标位置 
        self.cycle_completed = True                   # 是否完成当前轨迹
        self.start_time = None                        # 记录运动开始的时间

        # configuration of target trajectorypoint
        self.trajectory_points = trajectory_points
        self.validate_trajectory_points()
        self.trajectory_index = 0                     # 当前轨迹循环索引
    
    def validate_trajectory_points(self):
        """验证轨迹点的有效性"""
        for i in range(1, len(self.trajectory_points)):
            prev_position, prev_start_time, prev_duration = self.trajectory_points[i - 1]
            current_position, current_start_time, current_duration = self.trajectory_points[i]

            # 条件一: next_start_time 必须 >= 上一行的 start_time + duration
            if current_start_time < prev_start_time + prev_duration:
                raise ValueError(
                    f"第 {i + 1} 行的 next_start_time ({current_start_time}) 必须大于等于 "
                    f"第 {i} 行的 start_time ({prev_start_time}) 加 duration ({prev_duration})."
                )

            # 条件二: 当前点的速度必须小于设定的最大速度
            velocities = [
                abs(current_position[j] - prev_position[j]) / current_duration
                for j in range(2)                       # 增加从站
            ]
            if any(v > MAX_VELOCITY for v in velocities):
                raise ValueError(
                    f"第 {i + 1} 行的速度 ({velocities}) 超过最大速度 ({MAX_VELOCITY}). "
                    f"请调整 position 或 duration."
                )

    def listener_callback(self, msg):
        """订阅回调函数，更新当前电机位置"""
        self.current_position = list(msg.position[:2])  # 如有两个关节电机 # 增加从站
        self.get_logger().info(f"当前位置: {self.current_position}")
        logging.info(f"当前位置: {self.current_position}")

        # 判断运动是否完成
        if self.current_state == "MOVING":
            current_time = time.time() - self.start_time
            target_time, _ = self.trajectory_points[self.trajectory_index][1:3]
            if all(abs(self.current_position[i] - self.target_position[i]) < 4 for i in range(2)) and current_time >= target_time:
                self.get_logger().info("运动已完成.")
                logging.info("运动已完成.")
                self.cycle_completed = True
                self.current_state = "WAIT"

    def publish_trajectory(self):
        """发布当前周期的目标轨迹"""
        next_position, next_start_time, duration = self.trajectory_points[self.trajectory_index]

        # 发布轨迹消息
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()

        # 电机名称需要与URDF定义一致, 增加从站
        msg.joint_names = ['joint_0', 'joint_1']  
        point = JointTrajectoryPoint()
        
        point.positions = next_position
        point.time_from_start = rclpy.duration.Duration(seconds=duration).to_msg()
        msg.points.append(point)
        
        self.previous_target_position = self.target_position
        self.target_position = next_position
        self.publisher.publish(msg)
        self.get_logger().info(f"Published trajectory points: {next_position} at time {next_start_time + duration:.1f}s")
        logging.info(f"Published trajectory points: {next_position} at time {next_start_time + duration:.1f}s")
        
        # 记录运动开始时间
        self.start_time = time.time() - next_start_time
        
        # 更新轨迹索引
        self.trajectory_index = (self.trajectory_index + 1) % len(self.trajectory_points)
        self.current_state = "MOVING"

    def timer_callback(self):
        """周期性状态检查，并决定是否发布新轨迹"""
        if self.cycle_completed:
            self.cycle_completed = False
            self.publish_trajectory()

def main(args=None):
    rclpy.init(args=args)
    trajectory_file = 'trajectory_multipoints.txt'
    try:
        trajectory_points = load_trajectory_points(trajectory_file)
        node = DCControlNode(trajectory_points)
        rclpy.spin(node)
    except (ValueError, FileNotFoundError) as e:
        print(f"轨迹配置错误: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()