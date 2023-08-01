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
        self.create_subscription(String, "/input", self.callback_input, 10)
        timer_period = 3
        self.create_timer(timer_period, self.utterance_setup)

        self.agent1 = "poke_blue"
        self.agent2 = "poke_green"
        self.agent3 = "poke_yellow"
        self.agent_list = [self.agent1, self.agent2, self.agent3]
        self.human_text = ""

    def utterance_setup(self):
        w = Wizavo(wavname="shakai_4")

        #テキストファイル読み込み
        filedir_path = os.environ["HOME"] + "/his_ws/src/pokebo_cube/textfile/*"
        text_path = glob.glob(filedir_path)
        text_path = text_path[0]
        with open(text_path) as f:
            self.text_list = f.readlines()

        #ゴリ押し
        time.sleep(3)

        #utterance_core正常動作確認
        title = self.text_list.pop(0)
        title = title.split(" ")[0]
        utt = f"今からね、{title}、話すよ"
        wav = w.create_wavfile(utt)
        w.speak(wav)

        msg = String()
        msg.data = 'ready'
        self.state.publish(msg)
        time.sleep(4)

        self.utterance_manager()
    
    def utterance_manager(self):
        self.utterance_type = "normal"
        leader = self.agent1
        for i in range(len(self.text_list)):
            if i == 0:
                #発話文の先頭行
                speaker = random.choice(self.agent_list)
                text = self.text_list.pop(0).split(",")[0]
                msg = String()
                msg.data = f"{speaker}:{text}"
                self.sentense.publish(msg)

                leader = speaker
                self.utterance_type = self.next_pattern()
                time.sleep(4)
            else:
                if self.utterance_type == "normal":
                    speaker = leader
                    text = self.text_list.pop(0).split(",")[0]
                    msg = String()
                    msg.data = f"{speaker}:{text}"
                    self.sentense.publish(msg)

                    self.utterance_type = self.next_pattern()
                    time.sleep(4)
                elif self.utterance_type == "response":
                    speaker_list = [agent for agent in self.agent_list if agent != leader]
                    speaker = random.choice(speaker_list)
                    text = self.text_list.pop(0).split(",")[0]
                    msg = String()
                    msg.data = f"{speaker}:{text}"
                    self.sentense.publish(msg)

                    self.utterance_type = self.next_pattern()
                    time.sleep(4)
                elif self.utterance_type == "filler":
                    speaker = leader
                    text = self.text_list.pop(0).split(",")[0]
                    msg = String()
                    msg.data = f"{speaker}:{text}"
                    self.sentense.publish(msg)

                    self.utterance_type = self.next_pattern()
                    time.sleep(4)
                elif self.utterance_type == "back_channeling":
                    speaker_list = [agent for agent in self.agent_list if agent != leader]
                    speaker = random.choice(speaker_list)
                    text = self.text_list.pop(0).split(",")[0]
                    msg = String()
                    msg.data = f"{speaker}:{text}"
                    self.sentense.publish(msg)

                    leader = speaker
                    self.utterance_type = self.next_pattern()
                    time.sleep(6)

                elif self.utterance_type == "forget":
                    speaker = leader
                    tmp = self.text_list.pop(0).split(",")
                    text = random.choice(
                        ["えーと、あのー、", "うーんとね、", "あのー、そのー、"]
                    )
                    text += tmp[1]
                    answer_list = tmp[2].split(" ")
                    console.log(answer_list)
                    msg = String()
                    msg.data = f"{speaker}:{text}"
                    self.sentense.publish(msg)
                    time.sleep(8)
                    self.omoidasita = False
                    for i in range(2):
                        console.log(f"human_text:{self.human_text}")
                        if self.human_text in answer_list:
                            text = random.choice(
                                ["そうだーそれそれー!", "それだ!"]
                            )
                            text += self.text_list.pop(0)
                            msg = String()
                            msg.data = f"{speaker}:{text}"
                            self.sentense.publish(msg)
                            self.utterance_type = self.next_pattern()
                            self.omoidasita = True
                            time.sleep(6)
                            break
                        else:
                            speaker_list = [agent for agent in self.agent_list if agent != leader]
                            speaker = random.choice(speaker_list)
                            text = random.choice(
                                ["なんだっけ?", "うーんとー、"]
                            )
                            msg = String()
                            msg.data = f"{speaker}:{text}"
                            self.sentense.publish(msg)
                            time.sleep(6)
                    if not self.omoidasita:
                        speaker = leader
                        text = f"思い出した!、{self.text_list.pop(0)}"
                        msg = String()
                        msg.data = f"{speaker}:{text}"
                        self.sentense.publish(msg)
                        self.utterance_type = self.next_pattern()
                        time.sleep(6)





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
    
    def callback_input(self, msg):
        self.human_text = msg.data
        console.log(f"callback => human_text:{self.human_text}")


        

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