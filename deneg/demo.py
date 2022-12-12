#!/usr/bin/env python3

import time
import argparse

from deneg.deneg import DeNeg, Evaluator, Type
from deneg.examples import path_conflicts_proposals
from deneg.agent import Agent

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
