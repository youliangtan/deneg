#!/usr/bin/env python3

from deneg import DeNeg, Evaluator, Type
import time
import argparse
import random

from examples import path_conflicts_proposals

#############################################################

class Agent(DeNeg):
    def __init__(self, name, debug):
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
        print(f"\n ####################################### \n"
              f"{req.id}: [{self.name}] received assignemnt: [{req.content}]"
              f"\n         proposel: [{proposal}]"
              f"\n ####################################### \n")
        return True

def main(args):

    # if multi agent mode is selected, add 2 more agents, total 3
    debug = not args.prod
    a1 = Agent(args.agent, debug)

    if args.multi:
        a2 = Agent("agent2", debug)
        a3 = Agent("agent3", debug)

    time.sleep(1) # warm up

    # path resolution or task allocation
    if args.path_res:
        a1.submit("req2", {'desc': "path conflict at t=4,8"}, Type.PATH_RESOLUTION)
    elif args.task_alloc:
        a1.submit("req1", {'desc': "go flush the toilet"}, Type.TASK_ALLOCATION)

    # if user enabled spinning
    if args.spin:
        a1.spin()
    else:
        if args.path_res:
            time.sleep(25)
        else:
            time.sleep(12)

    # if multi agent mode is selected
    if args.multi:
        a2.shutdown()
        a3.shutdown()

    a1.shutdown()

    print("Bye peeps!")

#############################################################

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-a", "--agent", type=str, default='agent1',
                        help="agent name")
    # parser.add_argument("-c", "--cost", type=float, default=0.0,
    #                     help="predetermined cost")
    parser.add_argument("--path_res", action="store_true",
                        help='Run path conflict resolution example')
    parser.add_argument("--task_alloc", action="store_true",
                        help='Run task allocation example')
    parser.add_argument("--multi", action="store_true",
                        help='Run multi users test case')
    parser.add_argument("--spin", action="store_true",
                        help='Spin the node until ctrl-c shutdown')
    parser.add_argument("--prod", action="store_true",
                        help='Run in non-debug mode')
    args = parser.parse_args()

    main(args)
