from deneg import DeNeg
import time

import argparse

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('integers', metavar='N', type=int, nargs='+',
                    help='an integer for the accumulator')
parser.add_argument('--sum', dest='accumulate', action='store_const',
                    const=sum, default=max,
                    help='sum the integers (default: find the max)')

#############################################################

class Agent(DeNeg):
    def __init__(self, name):
        super().__init__(name)
    
    def receive_alert(self, id, content):
        print(f" {self.name} received alert with id {id}")
        return True

def main(agent_name):
    a = Agent(agent_name)
    a.send_alert("task2", {})

    # a2 = Agent("agent2")
    # a1 = Agent("agent1")
    # a1.send_alert("task1", {})
    # a.send_alert("task2", {})
    time.sleep(10)
    # a2.send_alert("task3", {})
    # time.sleep(1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-a", "--agent", type=str, default='agt',
                        help="agent name")
    args = parser.parse_args()
    main(args.agent)
