#!/usr/bin/env python3

import time
import argparse
import random

from deneg.deneg import DeNeg, Evaluator, Type
from deneg.examples import path_conflicts_proposals

from rclpy.utilities import remove_ros_args

##############################################################################

class Agent(DeNeg):
    def __init__(self, name, debug=True):
        super().__init__(name, debug)
        self.name = name
    
    def receive_alert(self, req):
        print(f"   received alert with id {req.id}, with type {req.type}")
        return True

    def proposal_submission(self, req, round):

        if req.type == Type.PATH_RESOLUTION:
            sample_proposals = path_conflicts_proposals(round)
            return sample_proposals[self.name]

        else:
            cost = random.uniform(1.0, 10.0)
            current_cost = random.uniform(5.0, 10.0)
            print(f"\n ======================================= \n"
                f"{req.id}: [{self.name}] "
                f"propose cost: [{cost}], current_cost: [{current_cost}]"
                f"\n ======================================= \n")
            return {"cost": cost, "current_cost": current_cost }

    def concession(self, req, round, results):
        print(f"{req.id}: received results {results}")

        if len(results) != len(self.participants()):
            print(f"{req.id}: not all participants have results")
            return False
        return True

    def round_table(self, req, round: int, all_proposals):
        if req.type == Type.PATH_RESOLUTION:
            res = Evaluator.PathConflictEvaluater(all_proposals)
            # print(res)
            return res
        else:
            return Evaluator.LowestCostEvaluater(all_proposals)

    def assignment(self, req, proposal):
        print(f"\n ####################################### \n"
              f"{req.id}: [{self.name}] received assignemnt: [{req.content}]"
              f"\n         proposel: [{proposal}]"
              f"\n ####################################### \n")
        return True

##############################################################################
# sample usage
def test():
    a1 = Agent("agent1")
    a2 = Agent("agent2")
    a3 = Agent("agent3")
    a1.submit("req1", {})
    time.sleep(12)
    a1.shutdown()
    a2.shutdown()
    a3.shutdown()
    print("Done Bye")

if __name__ == '__main__':
    test()
