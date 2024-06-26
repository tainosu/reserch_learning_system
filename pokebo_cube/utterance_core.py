# File              : utterance_core.py
# Author            : Taichi Sekikawa <sekikawa.taichi.vf@tut.jp>
# Date              : 2023 07/29
# Last Modified Date: 2024 05/10
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
        # 発話文を送信
        self.sentense = self.create_publisher(String, '/sentense', 1)
        # エージェントの発話状態を送信
        self.state = self.create_publisher(String, '/state', 1)
        # self.forget_word = self.create_publisher(String, '/get_word', 1)
        # 人の発話を受信
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
        self.wizavo = Wizavo(wavname="cube")

        #テキストファイル読み込み
        filedir_path = os.environ["HOME"] + "/chi_ws/src/pokebo_cube/textfile/*"
        text_path = glob.glob(filedir_path)
        text_path = text_path[1]    #0:tenki, 1:aichi, 2:kanji
        with open(text_path) as f:
            self.text_list = f.readlines()
        self.line_num = 0   #会話中の行数
        self.text_num = len(self.text_list) #テキスト全体の行数
        # console.log(self.text_list)

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
    
    
    #人の発話を受け取る（常に動いている）
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
            console.log(text)
            self.line_num += 1
            msg = String()
            msg.data = f"{self.speaker}:{text}:{self.utterance_type}"
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
                msg.data = f"{self.speaker}:{text}:{self.utterance_type}"
                self.sentense.publish(msg)
                self.utterance_type = self.next_pattern()
            
            elif self.utterance_type == "response":
                speaker_list = [agent for agent in self.agent_list if agent != self.leader]
                self.speaker = random.choice(speaker_list)
                text = self.text_list[self.line_num].split(",")[0]
                self.line_num += 1
                msg = String()
                msg.data = f"{self.speaker}:{text}:{self.utterance_type}"
                self.sentense.publish(msg)
                self.utterance_type = self.next_pattern()
            
            elif self.utterance_type == "filler":
                self.speaker = self.leader
                text = self.text_list[self.line_num].split(",")[0]
                self.line_num += 1
                msg = String()
                msg.data = f"{self.speaker}:{text}:{self.utterance_type}"
                self.sentense.publish(msg)
                self.utterance_type = self.next_pattern()
              
            elif self.utterance_type == "back_channeling":
                speaker_list = [agent for agent in self.agent_list if agent != self.leader]
                self.speaker = random.choice(speaker_list)
                text = self.text_list[self.line_num].split(",")[0]
                self.line_num += 1
                msg = String()
                msg.data = f"{self.speaker}:{text}:{self.utterance_type}"
                self.sentense.publish(msg)
                self.leader = self.speaker
                self.utterance_type = self.next_pattern()
            
            elif self.utterance_type == "forget":
                self.forget_mode()
    
    # 物忘れ状態の動作
    def forget_mode(self):
        forget_types = {
            "first": self.handle_first_forget,
            "second": self.handle_subsequent_forgets,
            "third": self.handle_subsequent_forgets,
            "forth": self.handle_subsequent_forgets,
            "fifth": self.handle_subsequent_forgets,
            "sixth": self.handle_sixth_forget
        }
        forget_handler = forget_types.get(self.forget_type, self.handle_first_forget)
        forget_handler()

    def handle_first_forget(self):
        self.speaker = self.leader
        tmp = self.text_list[self.line_num].split(",")
        self.line_num += 1
        text = tmp[1]
        self.answer_list = tmp[2].split(" ")
        console.log(self.answer_list)
        msg = String()
        msg.data = f"{self.speaker}:{text}:{self.utterance_type}"
        self.sentense.publish(msg)
        self.forget_type = "second"

    def handle_subsequent_forgets(self):
        time.sleep(1)
        # 人の発話が正解だった場合
        if self.human_text in self.answer_list:
            self.speaker = self.leader
            text = random.choice(["それそれー!、", "それだ!、", "そうだったー!、"])
            text += self.text_list[self.line_num].split(",")[0]
            self.line_num += 1
            msg = String()
            msg.data = f"{self.speaker}:{text}:{self.utterance_type}"
            self.sentense.publish(msg)
            self.forget_type = "first"
            self.utterance_type = self.next_pattern()
            time.sleep(4)
        else:
            self.give_hint()

    def handle_sixth_forget(self):
        time.sleep(1)
        # 強制的に思い出す
        if self.human_text in self.answer_list:
            self.speaker = self.leader
            text = random.choice(["それそれー!、", "それだ!、", "そうだったー!、"])
            text += self.text_list[self.line_num].split(",")[0]
            self.line_num += 1
            msg = String()
            msg.data = f"{self.speaker}:{text}:{self.utterance_type}"
            self.sentense.publish(msg)
            self.forget_type = "first"
            self.utterance_type = self.next_pattern()
            time.sleep(4)
        else:
            self.speaker = self.leader
            text = f"思い出した!、{self.text_list[self.line_num]}"
            self.line_num += 1
            msg = String()
            msg.data = f"{self.speaker}:{text}:{self.utterance_type}"
            self.sentense.publish(msg)
            self.forget_type = "first"
            self.utterance_type = self.next_pattern()
            time.sleep(4)

    # 答えの先頭文字をヒントとして話す
    def give_hint(self):
        hint = self.answer_list[0][0]
        console.log(f"ヒント:{hint}")
        speaker_list = [agent for agent in self.agent_list if agent != self.speaker]
        self.speaker = random.choice(speaker_list)
        hint_sentence = hint + "、から始まる気がするなあ"
        text = random.choice(["なんだっけ?", "うーんとー", "思い出せないなあ", hint_sentence])
        msg = String()
        msg.data = f"{self.speaker}:{text}:{self.utterance_type}"
        self.sentense.publish(msg)
        self.advance_forget_stage()

    def advance_forget_stage(self):
        forget_stages = ["second", "third", "forth", "fifth", "sixth"]
        current_stage = forget_stages.index(self.forget_type)
        if current_stage < len(forget_stages) - 1:
            self.forget_type = forget_stages[current_stage + 1]

    
        

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