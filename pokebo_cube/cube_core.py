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
from pywizavo import Wizavo
import random

console = Console()

class Cube(Node):
    def __init__(self):
        super().__init__('cube_core')
        self.name = self.get_name()
        self.wizavo = Wizavo(self.name)

        self.declare_parameter('assign_cube', 'hoge')
        cube_name = self.get_parameter('assign_cube').value
        console.log(f'assign cube: {self.name} => {cube_name}')
        self.behavior = self.create_publisher(String, f'/pokebo_cube/{cube_name}/dc/behavior', 2)

        #cubeの位置関係
        blue_lookat = {'poke_green': 'left', 'poke_yellow': 'forward-left', 'human': 'upper-right'}
        green_lookat = {'poke_blue': 'right', 'poke_yellow': 'left', 'human': "up"}
        yellow_lookat = {'poke_green': 'right', 'poke_blue': 'forward-right', 'human': 'upper-left'}
        self.lookat = {'poke_green': green_lookat, 'poke_yellow': yellow_lookat, 'poke_blue': blue_lookat}

        self.create_subscription(String, '/state', self.callback_state, 1)
        self.create_subscription(String, '/sentense', self.callback_sentense, 1)
        self.pub_speak = self.create_publisher(String, '/pokebo_cube/speak', 2)
        self.create_subscription(String, '/pokebo_cube/speak', self.callback_speak, 1)

        #プログラム正常起動確認
        msg = String()
        msg.data = 'up'
        self.behavior.publish(msg)
        console.log(f'[magenta]{self.name}[/] > あわわ')
        wav = self.wizavo.create_wavfile('あわわ')
        self.wizavo.speak(wav)
        msg = String()
        msg.data = 'default'
        self.behavior.publish(msg)
    
    def callback_state(self, msg):
        if msg.data == 'ready':
            console.log(f'server ready')
            msg = String()
            msg.data = 'nod'
            self.behavior.publish(msg)
    
    def callback_sentense(self, msg):
        data = msg.data.split(":")
        name = data[0]
        text = data[1]
        flag = data[2]
        # console.log(f"name:{name}, text:{text}, flag:{flag}")

        if self.name == name:
            if flag == "forget":
                com = self.lookat[self.name]['human']
                console.log(f"behavior -- {com}")
            else:
                com = "up"
            msg = String()
            msg.data = com
            self.behavior.publish(msg)
            
            msg = String()
            msg.data = f"{self.name}:start:{flag}"
            self.pub_speak.publish(msg)
            wav = self.wizavo.create_wavfile(text)
            self.wizavo.speak(wav)
            msg = String()
            msg.data = f"{self.name}:end:{flag}"
            self.pub_speak.publish(msg)
    
    def callback_speak(self, msg):
        data = msg.data.split(":")
        name = data[0]
        state = data[1]
        flag = data[2]

        #publishしてきたエージェントが自分自身だった場合、何もしない
        if name == self.name:
            return
        
        if state == "start":
            com = self.lookat[self.name][name]
            console.log(f"behavior -- {com}")
            msg = String()
            msg.data = com
            self.behavior.publish(msg)
        else:
            if flag == "forget":
                return
            com = random.choice(
                ["nod", "nodnod", "shallow-nod"]
            )
            console.log(f"behavior == {com}")
            msg = String()
            msg.data = com
            self.behavior.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    cube = Cube()
    console.log(f'run cube node: {cube.name}')
    try:
        rclpy.spin(cube)
    except KeyboardInterrupt:
        console.log('KeyboardInterrrupt')
    cube.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
