#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ParametersBasicNode(Node):
    """
    创建一个ParametersBasicNode节点，并在初始化时输出一个话
    """
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info(f"节点已启动：{name}!")
        # 声明参数
        self.declare_parameter('rcl_log_level', 0)
        # 获取参数
        log_level = self.get_parameter("rcl_log_level").value
        # 设置参数
        self.get_logger().set_level(log_level)
        # 定时修改
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        """定时器回调函数"""
        # 获取参数
        log_level = self.get_parameter("rcl_log_level").value
        # 设置参数
        self.get_logger().set_level(log_level)
        print(
            f"========================{log_level}=============================")
        self.get_logger().debug("我是DEBUG级别的日志，我被打印出来了!")
        self.get_logger().info("我是INFO级别的日志，我被打印出来了!")
        self.get_logger().warn("我是WARN级别的日志，我被打印出来了!")
        self.get_logger().error("我是ERROR级别的日志，我被打印出来了!")
        self.get_logger().fatal("我是FATAL级别的日志，我被打印出来了!")
        

def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = ParametersBasicNode("parameters_basic")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy

