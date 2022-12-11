#!/usr/bin/env python3

from deneg import DeNeg
import time
import argparse
import random

#############################################################

class Agent(DeNeg):
    def __init__(self, name):
        super().__init__(name)
    
    def receive_alert(self, id, content):
        print(f"   received alert with id {id}")
        return True

    def proposal_submission(self, round):
        return {"cost": random.uniform(1.0, 10.0) }

    def concession(self, results):
        print(f"   received results {results}")
        return True

    # def round_table(self, round: int, other_proposals):
    #     return []

    def assignment(self, assignment):
        print(f"   received assignment {assignment}")
        return True

def main(agent_name):

    a1 = Agent("agent1")
    a2 = Agent("agent2")
    a3 = Agent("agent3")
    a1.send_alert("task1", {})

    time.sleep(4.5)

    a1.shutdown()
    a2.shutdown()
    a3.shutdown()

    print("Bye peeps!")

#############################################################

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-a", "--agent", type=str, default='agt',
                        help="agent name")
    args = parser.parse_args()
    main(args.agent)
