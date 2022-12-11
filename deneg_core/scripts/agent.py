#!/usr/bin/env python3

from deneg import DeNeg, Evaluator
import time
import argparse
import random

#############################################################

class Agent(DeNeg):
    def __init__(self, name):
        super().__init__(name)
        self.name = name
    
    def receive_alert(self, id, content):
        print(f"   received alert with id {id}")
        return True

    def proposal_submission(self, id, round):
        cost = random.uniform(1.0, 10.0)
        print(f" \n ======================================= \n"
              f"{id}: [{self.name}] propose cost: [{cost}]"
              "\n ======================================= \n")
        return {"cost": cost }

    def concession(self, id, results):
        print(f"{id}: received results {results}")
        return True

    def round_table(self, id, round: int, other_proposals):
        return Evaluator.LowestCostEvaluater(other_proposals)

    def assignment(self, id, assignment):
        print(f" \n ======================================= \n"
              f"{id}: [{self.name}] received assignemnt: [{assignment}]"
              "\n ======================================= \n")
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
