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
        cost = random.uniform(1.0, 10.0)
        print(f" \n ======================================= \n \
            [{self.name}] propose cost: [{cost}] \
            \n ======================================= \n")
        return {"cost": cost }

    def concession(self, results):
        print(f"   received results {results}")
        return True

    # def round_table(self, round: int, other_proposals):
    #     return []

    def assignment(self, assignment):
        print(f" \n ======================================= \n \
            [{self.name}] received assignment [{assignment}] \
            \n ======================================= \n")
        return True

def main(agent_name):

    a1 = Agent("agent1")
    a2 = Agent("agent2")
    a3 = Agent("agent3")
    a1.send_alert("task1", {})

    # a1.spin()
    time.sleep(10)

    a1.shutdown()
    a2.shutdown()
    a3.shutdown()

    print("Bye peeps!")

#############################################################

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-a", "--agent", type=str, default='agt',
                        help="agent name")
    parser.add_argument("-c", "--cost", type=float, default=0.0,
                        help="predetermined cost")
    args = parser.parse_args()
    main(args.agent)
