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
import time

#python
from rich.console import Console
import glob
import os
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
        timer_period = 4.5
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
        filedir_path = os.environ["HOME"] + "/chi_ws/src/pokebo_cube/textfile/*"
        text_path = glob.glob(filedir_path)
        text_path = text_path[1]
        with open(text_path) as f:
            self.text_list = f.readlines()
        self.line_num = 0   #現在の行数
        self.text_num = len(self.text_list) #テキストの行数

        #cube起動待ち
        time.sleep(3.5)

        #utterance_core正常起動確認
        title = self.text_list.pop(0)
        title = title.split(" ")[0]
        utt = f"今からね、{title}、話すよ"
        wav = self.wizavo.create_wavfile(utt)
        self.wizavo.speak(wav)
        msg = String()
        msg.data = 'ready'
        self.state.publish(msg)
        #wizavo出力待ち
        time.sleep(2)
    
    
    #cubeが話し終わったタイミングで人の発話を受け取る
    def callback_input(self, msg):
        self.human_text = msg.data
        console.log(f"callback => human_text:{self.human_text}")
    
    #utterance_typeを管理
    def next_pattern(self):
        #テキストの末尾までたどり着いた場合
        if self.line_num == self.text_num - 1:
            next = "normal"
            self.head_flag = True
            self.line_num = 0
            return next
        
        text = self.text_list[self.line_num]
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
    
    #テキストを一行読み取り、発話するエージェントを決定
    def utterance_manager(self):
        if self.head_flag:
            #発話文の先頭行
            self.speaker = random.choice(self.agent_list)
            text = self.text_list[self.line_num].split(",")[0]
            self.line_num += 1
            msg = String()
            msg.data = f"{self.speaker}:{text}"
            self.sentense.publish(msg)
            self.leader = self.speaker
            self.utterance_type = self.next_pattern()

            self.head_flag = False
        else:
            if self.utterance_type == "normal":
                self.speaker = self.leader
                text = self.text_list[self.line_num].split(",")[0]
                self.line_num += 1
                msg = String()
                msg.data = f"{self.speaker}:{text}"
                self.sentense.publish(msg)
                self.utterance_type = self.next_pattern()
            
            elif self.utterance_type == "response":
                speaker_list = [agent for agent in self.agent_list if agent != self.leader]
                self.speaker = random.choice(speaker_list)
                text = self.text_list[self.line_num].split(",")[0]
                self.line_num += 1
                msg = String()
                msg.data = f"{self.speaker}:{text}"
                self.sentense.publish(msg)
                self.utterance_type = self.next_pattern()
            
            elif self.utterance_type == "filler":
                self.speaker = self.leader
                text = self.text_list[self.line_num].split(",")[0]
                self.line_num += 1
                msg = String()
                msg.data = f"{self.speaker}:{text}"
                self.sentense.publish(msg)
                self.utterance_type = self.next_pattern()
              
            elif self.utterance_type == "back_channeling":
                speaker_list = [agent for agent in self.agent_list if agent != self.leader]
                self.speaker = random.choice(speaker_list)
                text = self.text_list[self.line_num].split(",")[0]
                self.line_num += 1
                msg = String()
                msg.data = f"{self.speaker}:{text}"
                self.sentense.publish(msg)
                self.leader = self.speaker
                self.utterance_type = self.next_pattern()

                #time.sleep(6)
            
            elif self.utterance_type == "forget":
                self.forget_mode()
    
    def forget_mode(self):
        if self.forget_type == "first":
            self.speaker = self.leader
            tmp = self.text_list[self.line_num].split(",")
            self.line_num += 1
            text = tmp[1]
            self.answer_list = tmp[2].split(" ")
            console.log(self.answer_list)
            msg = String()
            msg.data = f"{self.speaker}:{text}"
            self.sentense.publish(msg)
            self.forget_type = "second"

            #time.sleep(8.5)
          
        
        elif self.forget_type == "second" or self.forget_type == "third" or self.forget_type == "forth":
            time.sleep(1)
            if self.human_text in self.answer_list:
                self.speaker = self.leader
                text = random.choice(
                    ["それそれー!、", "それだ!、", "そうだったー!、"]
                )
                text += self.text_list[self.line_num].split(",")[0]
                self.line_num += 1
                msg = String()
                msg.data = f"{self.speaker}:{text}"
                self.sentense.publish(msg)
                self.forget_type = "first"
                self.utterance_type = self.next_pattern()
        
            else:
                speaker_list = [agent for agent in self.agent_list if agent != self.speaker]
                self.speaker = random.choice(speaker_list)
                text = random.choice(
                    ["なんだっけ?", "なんだっけ?", "うーんとー", "うーんとー", "思い出せないなあ"]
                )
                msg = String()
                msg.data = f"{self.speaker}:{text}"
                self.sentense.publish(msg)

                if self.forget_type == "second":
                    self.forget_type = "third"
                elif self.forget_type == "third":
                    self.forget_type = "forth"
                elif self.forget_type == "forth":
                    self.forget_type = "fifth"

            #time.sleep(6.5)    

        
        elif self.forget_type == "fifth":
            time.sleep(1)
            if self.human_text in self.answer_list:
                self.speaker = self.leader
                text = random.choice(
                    ["それそれー!、", "それだ!、", "そうだったー!、"]
                )
                text += self.text_list[self.line_num].split(",")[0]
                self.line_num += 1
                msg = String()
                msg.data = f"{self.speaker}:{text}"
                self.sentense.publish(msg)
                self.forget_type = "first"
                self.utterance_type = self.next_pattern()
        
            else:
                self.speaker = self.leader
                text = f"思い出した!、{self.text_list[self.line_num]}"
                self.line_num += 1
                msg = String()
                msg.data = f"{self.speaker}:{text}"
                self.sentense.publish(msg)
                self.forget_type = "first"
                self.utterance_type = self.next_pattern()

            #time.sleep(6.5)
        

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