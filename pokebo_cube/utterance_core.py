#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# File              : cube_core.py
# Author            : Taichi Sekikawa <sekikawa.taichi.vf@tut.jp>
# Date              : 2023 07/29
# Last Modified Date: 2023 07/29
# Last Modified By  : Taichi Sekikawa <sekikawa.taichi.vf@tut.jp>

#ROS2
import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from rclpy.clock import Clock, ClockType
from std_msgs.msg import String

#python
from rich.console import Console
import glob
import os
import time
import random

from pywizavo import Wizavo

console = Console()

class UtteranceCore(Node):
    def __init__(self):
        super().__init__('utterance_core')
        self.sentense = self.create_publisher(String, '/sentense', 1)
        self.state = self.create_publisher(String, '/state', 1)
        self.forget_word = self.create_publisher(String, '/get_word', 1)
        self.create_subscription(String, "/input", self.callback_input, 1)
        timer_period = 6
        self.create_timer(timer_period, self.utterance_manager)

        self.agent1 = "poke_blue"
        self.agent2 = "poke_green"
        self.agent3 = "poke_yellow"
        self.agent_list = [self.agent1, self.agent2, self.agent3]
        self.human_text = ""

        self.head_flag = True
        self.utterance_type = "normal"
        self.forget_type = "first"
        self.leader = self.agent1
        self.speaker = self.agent1
        self.answer_list = []

        self.wizavo = Wizavo(wavname="shakai_4")

        #テキストファイル読み込み
        filedir_path = os.environ["HOME"] + "/his_ws/src/pokebo_cube/textfile/*"
        text_path = glob.glob(filedir_path)
        text_path = text_path[0]
        with open(text_path) as f:
            self.text_list = f.readlines()
        #ゴリ押し
        self.stop_time = Clock(clock_type=ClockType.ROS_TIME).now() + Duration(seconds=3)
        while(1):
            if Clock(clock_type=ClockType.ROS_TIME).now() > self.stop_time:
                console.log(f"最初のクロックぬけた")
                break
        
        #utterance_core正常起動確認
        title = self.text_list.pop(0)
        title = title.split(" ")[0]
        utt = f"今からね、{title}、話すよ"
        wav = self.wizavo.create_wavfile(utt)
        self.wizavo.speak(wav)

        msg = String()
        msg.data = 'ready'
        self.state.publish(msg)
        #sleep
        self.stop_time = Clock(clock_type=ClockType.ROS_TIME).now() + Duration(seconds=3)
        while(1):
            if Clock(clock_type=ClockType.ROS_TIME).now() > self.stop_time:
                break


    
    def callback_input(self, msg):
        self.human_text = msg.data
        console.log(f"callback => human_text:{self.human_text}")

    
    def utterance_manager(self):
        if self.head_flag:
            #発話文の先頭行
            self.speaker = random.choice(self.agent_list)
            text = self.text_list.pop(0).split(",")[0]
            msg = String()
            msg.data = f"{self.speaker}:{text}"
            self.sentense.publish(msg)
            self.leader = self.speaker
            self.utterance_type = self.next_pattern()

            self.head_flag = False
            #sleep
            # self.stop_time = Clock(clock_type=ClockType.ROS_TIME).now() + Duration(seconds=3)
            # while(1):
            #     if Clock(clock_type=ClockType.ROS_TIME).now() > self.stop_time:
            #         break
        else:
            if self.utterance_type == "normal":
                self.speaker = self.leader
                text = self.text_list.pop(0).split(",")[0]
                msg = String()
                msg.data = f"{self.speaker}:{text}"
                self.sentense.publish(msg)
                self.utterance_type = self.next_pattern()
                #sleep
                # self.stop_time = Clock(clock_type=ClockType.ROS_TIME).now() + Duration(seconds=3)
                # while(1):
                #     if Clock(clock_type=ClockType.ROS_TIME).now() > self.stop_time:
                #         break
            elif self.utterance_type == "response":
                speaker_list = [agent for agent in self.agent_list if agent != self.leader]
                self.speaker = random.choice(speaker_list)
                text = self.text_list.pop(0).split(",")[0]
                msg = String()
                msg.data = f"{self.speaker}:{text}"
                self.sentense.publish(msg)
                self.utterance_type = self.next_pattern()
                #sleep
                # self.stop_time = Clock(clock_type=ClockType.ROS_TIME).now() + Duration(seconds=3)
                # while(1):
                #     if Clock(clock_type=ClockType.ROS_TIME).now() > self.stop_time:
                #         break
            elif self.utterance_type == "filler":
                self.speaker = self.leader
                text = self.text_list.pop(0).split(",")[0]
                msg = String()
                msg.data = f"{self.speaker}:{text}"
                self.sentense.publish(msg)
                self.utterance_type = self.next_pattern()
                # sleep
                # self.stop_time = Clock(clock_type=ClockType.ROS_TIME).now() + Duration(seconds=3)
                # while(1):
                #     if Clock(clock_type=ClockType.ROS_TIME).now() > self.stop_time:
                #         break
            elif self.utterance_type == "back_channeling":
                speaker_list = [agent for agent in self.agent_list if agent != self.leader]
                self.speaker = random.choice(speaker_list)
                text = self.text_list.pop(0).split(",")[0]
                msg = String()
                msg.data = f"{self.speaker}:{text}"
                self.sentense.publish(msg)
                self.leader = self.speaker
                self.utterance_type = self.next_pattern()
                # sleep
                # self.stop_time = Clock(clock_type=ClockType.ROS_TIME).now() + Duration(seconds=6)
                # while(1):
                #     if Clock(clock_type=ClockType.ROS_TIME).now() > self.stop_time:
                #         break
            elif self.utterance_type == "forget" and self.forget_type == "first":
                self.speaker = self.leader
                tmp = self.text_list.pop(0).split(",")
                text = random.choice(
                    ["えーと、あのー、", "うーんとね、", "あのー、そのー、"]
                )
                text += tmp[1]
                self.answer_list = tmp[2].split(" ")
                console.log(self.answer_list)
                msg = String()
                msg.data = f"{self.speaker}:{text}"
                self.sentense.publish(msg)
                self.forget_type = "second"
                # time.sleep(8)
                # self.stop_time = Clock(clock_type=ClockType.ROS_TIME).now() + Duration(seconds=8)
                # while(1):
                #     if Clock(clock_type=ClockType.ROS_TIME).now() > self.stop_time:
                #         break

                # msg = String()
                # msg.data = "request"
                # self.forget_word.publish(msg)
                # console.log(f"human_text:{self.human_text}")
            
            elif self.utterance_type == "forget" and self.forget_type == "second":
                if self.human_text in self.answer_list:
                    text = random.choice(
                        ["そうだーそれそれー!", "それだ!"]
                    )
                    text += self.text_list.pop(0)
                    msg = String()
                    msg.data = f"{self.speaker}:{text}"
                    self.sentense.publish(msg)
                    self.forget_type = "first"
                    self.utterance_type = self.next_pattern()
                    # sleep
                    # self.stop_time = Clock(clock_type=ClockType.ROS_TIME).now() + Duration(seconds=6)
                    # while(1):
                    #     if Clock(clock_type=ClockType.ROS_TIME).now() > self.stop_time:
                    #         break
                else:
                    speaker_list = [agent for agent in self.agent_list if agent != self.leader]
                    self.speaker = random.choice(speaker_list)
                    text = random.choice(
                        ["なんだっけ?", "うーんとー、"]
                    )
                    msg = String()
                    msg.data = f"{self.speaker}:{text}"
                    self.sentense.publish(msg)
                    self.forget_type = "third"
                    # sleep
                    # self.stop_time = Clock(clock_type=ClockType.ROS_TIME).now() + Duration(seconds=6)
                    # while(1):
                    #     if Clock(clock_type=ClockType.ROS_TIME).now() > self.stop_time:
                    #         break
            
            elif self.utterance_type == "forget" and self.forget_type == "third":
                if self.human_text in self.answer_list:
                    text = random.choice(
                        ["そうだーそれそれー!", "それだ!"]
                    )
                    text += self.text_list.pop(0)
                    msg = String()
                    msg.data = f"{self.speaker}:{text}"
                    self.sentense.publish(msg)
                    self.forget_type = "first"
                    self.utterance_type = self.next_pattern()
                    # sleep
                    # self.stop_time = Clock(clock_type=ClockType.ROS_TIME).now() + Duration(seconds=6)
                    # while(1):
                    #     if Clock(clock_type=ClockType.ROS_TIME).now() > self.stop_time:
                    #         break
                else:
                    speaker_list = [agent for agent in self.agent_list if agent != self.leader]
                    self.speaker = random.choice(speaker_list)
                    text = random.choice(
                        ["なんだっけ?", "うーんとー、"]
                    )
                    msg = String()
                    msg.data = f"{self.speaker}:{text}"
                    self.sentense.publish(msg)
                    self.forget_type = "forth"
                    # sleep
                    # self.stop_time = Clock(clock_type=ClockType.ROS_TIME).now() + Duration(seconds=6)
                    # while(1):
                    #     if Clock(clock_type=ClockType.ROS_TIME).now() > self.stop_time:
                    #         break
            elif self.utterance_type == "forget" and self.forget_type == "forth":
                self.speaker = self.leader
                text = f"思い出した!、{self.text_list.pop(0)}"
                msg = String()
                msg.data = f"{self.speaker}:{text}"
                self.sentense.publish(msg)
                self.forget_type = "first"
                self.utterance_type = self.next_pattern()
                # sleep
                # self.stop_time = Clock(clock_type=ClockType.ROS_TIME).now() + Duration(seconds=6)
                # while(1):
                #     if Clock(clock_type=ClockType.ROS_TIME).now() > self.stop_time:
                #         break




    def next_pattern(self):
        if len(self.text_list) == 0:
            next = "none"
            return next
        
        text = self.text_list[0]

        if "[masked]" in text:
            next = "forget"
        elif text[-2] == '.':
            next = "back_channeling"
        elif self.utterance_type == "normal":
            next = "response"
        elif self.utterance_type == "response":
            next = "filler"
        elif self.utterance_type == "filler":
            next = "normal"
        elif self.utterance_type == "back_channeling":
            next = "normal"
        elif self.utterance_type == "forget":
            next = "back_channeling"
        return next
    
    


        

def main(args=None):
    rclpy.init(args=args)
    utterance_core = UtteranceCore()
    try:
        rclpy.spin(utterance_core)
    except KeyboardInterrupt:
        console.log('KeyboardInterrrupt')
    utterance_core.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
