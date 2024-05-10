#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# File              : cube_core.py
# Author            : Taichi Sekikawa <sekikawa.taichi.vf@tut.jp>
# Date              : 2023 07/29
# Last Modified Date: 2023 10/10
# Last Modified By  : Taichi Sekikawa <sekikawa.taichi.vf@tut.jp>

#ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#python
from rich.console import Console

console = Console()

class HumanInput(Node):
    def __init__(self):
        super().__init__('human_input')
        self.input = ""
        self.pub_input = self.create_publisher(String, "/input", 10)
        self.create_subscription(String, "/listen", self.callback_listen, 1)
    
    # 音声認識結果を受け取るコールバック関数
    def callback_listen(self, msg):
        self.input = msg.data
        console.log(f"subscribe!---->{self.input}")
        msg = String()
        msg.data = self.input
        self.pub_input.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    human = HumanInput()
    console.log('run human_input node')
    try:
        rclpy.spin(human)
    except KeyboardInterrupt:
        console.log('KeyboardInterrrupt')
    human.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()