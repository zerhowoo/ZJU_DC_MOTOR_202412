# modified by Wang,Zihao at 2024.12.10 in Institute of Advanced Machines Zhejiang University

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from datetime import datetime
import time         
import logging     # 打印运动信息到文本文件中
import os          # 从文本文件中获取轨迹点信息 

# 获取当前日期和时间，格式化为文件名的一部分 
current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S") 
log_filename = f'log_book/output_{current_time}.txt' 
# 配置日志记录 
logging.basicConfig( filename=log_filename, level=logging.INFO, format='%(asctime)s - %(message)s' ) 

MAX_VELOCITY = 200000 # 最大速度限制，每秒

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
                    position, start_time, duration = map(float, line.split(','))
                    trajectory_points.append((position, start_time, duration))
                except ValueError:
                    raise ValueError(f"文件格式错误：{line} 需要是 'position, start_time, duration' 格式。")
    return trajectory_points

class DCControlNode(Node):
    def __init__(self, trajectory_points):
        super().__init__('dc_control')

        # 发布和订阅
        self.publisher = self.create_publisher(JointTrajectory, '/trajectory_controller/joint_trajectory', 10)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.listener_callback, 10)
        
        # 定时器
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 状态管理 
        self.current_state = "WAIT"           # 初始状态为等待 
        self.current_position = 0.0           # 当前位置
        self.target_position = 0.0            # 当前目标位置
        self.previous_target_position = None  # 上一个目标位置 
        self.cycle_completed = True           # 是否完成当前轨迹
        self.start_time = None                # 记录运动开始的时间

        # 目标轨迹配置（时间单位为秒，位置单位为自定义单位）(next_position, next_start_time, duration)
        self.trajectory_points = trajectory_points
        # [
        #     (0.0, 0.0, 5.0),                 # 在位置0.0           静止 5.0 秒
        #     (200000.0, 5.0, 8.0),            # 到位置200000.0      花了 8.0 秒
        #     (400000.0, 18.0, 6.0),           # 在位置200000.0 静止 5.0 秒，到位置400000.0 花了 8 秒
        #     (0.0, 30.0, 10.0),               # 在位置400000.0 静止 4.0 秒，到位置0.0 花了 10 秒
        # ]
        self.validate_trajectory_points() 
        self.trajectory_index = 0            # 当前轨迹循环索引
    
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

            # 条件二: 当前点的速度必须小于最大速度
            velocity = abs(current_position - prev_position) / current_duration
            if velocity > MAX_VELOCITY:
                raise ValueError(
                    f"第 {i + 1} 行的速度 ({velocity:.2f}) 超过最大速度 ({MAX_VELOCITY}). "
                    f"请调整 position 或 duration."
                )

    def listener_callback(self, msg):
        """订阅回调函数，更新当前电机位置"""
        self.current_position = msg.position[0]                                  # 假设有一个关节电机
        self.get_logger().info(f"当前位置: {self.current_position:.2f}")
        logging.info(f"当前位置: {self.current_position:.2f}")

        # 判断运动是否完成
        if self.current_state == "MOVING":
            current_time = time.time() - self.start_time                                                # 计算当前已过去的时间
            target_time, _ = self.trajectory_points[self.trajectory_index][1:3]                         # 获取目标时间
            if abs(self.current_position - self.target_position) < 4 and current_time >= target_time:   # 允许微小误差或时间到达
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
        msg.joint_names = ['joint_0']  # 电机名称需要与URDF定义一致
        point = JointTrajectoryPoint()
        
        point.positions = [next_position]
        point.time_from_start = rclpy.duration.Duration(seconds=duration).to_msg()
        msg.points.append(point)
        
        self.previous_target_position = self.target_position
        self.target_position = next_position
        self.publisher.publish(msg)
        self.get_logger().info(f"Published trajectory point: {next_position} at time {next_start_time + duration:.1f}s")
        logging.info(f"Published trajectory point: {next_position} at time {next_start_time + duration:.1f}s")
        
        # # 记录运动开始时间
        self.start_time = time.time() - next_start_time  # 调整为从当前时间开始计时
        
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
    trajectory_file = 'trajectory_points.txt'
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