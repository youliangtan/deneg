import random
from deneg_msgs.msg import Notify
from pydantic import BaseModel
from typing import List, Optional, Dict

#############################################################
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class fg:
    red = '\033[31m'
    green = '\033[32m'
    orange = '\033[33m'
    blue = '\033[34m'
    purple = '\033[35m'
    cyan = '\033[36m'
    lightgrey = '\033[37m'
    darkgrey = '\033[90m'
    lightred = '\033[91m'
    lightgreen = '\033[92m'
    yellow = '\033[93m'
    lightblue = '\033[94m'
    pink = '\033[95m'
    lightcyan = '\033[96m'

def random_color():
    return random.choice(
            [fg.red, fg.green, fg.orange, fg.blue, fg.purple, fg.cyan, fg.lightgrey, fg.darkgrey, fg.lightred, fg.lightgreen, fg.yellow, fg.lightblue, fg.pink, fg.lightcyan]
        )

#############################################################
class State:
    def __init__(self):
        self.state_seq = [
            Notify.JOIN,
            Notify.READY,
            Notify.NEGO,
            Notify.RANK,
            Notify.CONSENT,
            Notify.ASSIGNMENT,
        ]
        self.current_state_idx = 0

        self.bad_state = False
        self.bad_state_idx = -1
        self.bad_states = [
            Notify.ERROR,
            Notify.LEAVE,
        ]

    def next(self):
        if self.bad_state:
            return None
        self.current_state_idx += 1
        if self.current_state_idx >= len(self.state_seq):
            return self.state_seq[-1]
        return self.state_seq[self.current_state_idx]

    def current(self):
        if self.bad_state:
            return self.bad_states[self.bad_state_idx]
        if self.current_state_idx >= len(self.state_seq):
            return self.state_seq[-1]
        return self.state_seq[self.current_state_idx]
    
    def reset(self):
        self.current_state_idx = 0
        self.bad_state = False
    
    def error(self):
        self.bad_state = True
        self.bad_state_idx = 0

    def leave(self):
        self.bad_state = True
        self.bad_state_idx = 1
    
#############################################################

class Participant(BaseModel):
    state: int = Notify.JOIN
    name: str = ""
    proposal: Dict = {}
    assignment: Optional[Dict]= None
    ranking: List[str] = []
    data: int = 0
