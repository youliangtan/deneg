#!/usr/bin/env python3

from deneg import DeNeg, Evaluator, Type
import time
import argparse
import random

from examples import path_conflicts_proposals

#############################################################

class Agent(DeNeg):
    def __init__(self, name):
        super().__init__(name)
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

        # TODO: The line below is a test implementation for multiple rounds NEGO
        # if round == 0:
        #     return False
        return True

    def round_table(self, req, round: int, all_proposals):
        if req.type == Type.PATH_RESOLUTION:
            res = Evaluator.PathConflictEvaluater(all_proposals)
            # print(res)
            return res
        else:
            return Evaluator.LowestCostEvaluater(all_proposals)

    def assignment(self, req, proposal):
        print(f"\n ======================================= \n"
              f"{req.id}: [{self.name}] received assignemnt: [{req.content}]"
              f"\n         proposel: [{proposal}]"
              f"\n ======================================= \n")
        return True

def main(args):

    a1 = Agent("agent1")
    a2 = Agent("agent2")
    a3 = Agent("agent3")
    # a4 = Agent("agent4")

    if args.path:
        a1.submit("task2", {'desc': "deliver me a coke"}, Type.PATH_RESOLUTION)
        time.sleep(20)
    else:
        a1.submit("task1", {'desc': "go flush the toilet"}, Type.TASK_ALLOCATION)
        time.sleep(11)

    # a1.spin()

    a1.shutdown()
    a2.shutdown()
    a3.shutdown()
    # a4.shutdown()

    print("Bye peeps!")

#############################################################

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-a", "--agent", type=str, default='agt',
                        help="agent name")
    parser.add_argument("-c", "--cost", type=float, default=0.0,
                        help="predetermined cost")
    parser.add_argument("--path", action="store_true",
                        help='Run path conflict resolution example')
    args = parser.parse_args()
    main(args)
