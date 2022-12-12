# DeNeg Library

Library for decentralized negotiation system

**WORK IN PROGRESS**

DeNeg is a python library for user to implement decentralized negotiation system on each agent. This library should be implemented on each agent in the system.

Dependencies
 - ros2 (Tested on humble)

Compilation
```bash
cd ros2_ws
colcon build
```

## Example

```bash
source install/setup.bash

# run example with task allocation
python3 deneg/deneg_core/scripts/agent.py

# Path resolution example
python3 deneg/deneg_core/scripts/agent.py --path
```

## Code Snippet

Simple example of using DeNeg for task allocation 

```py
class Agent(DeNeg):
    def __init__(self, name):
        super().__init__(name)
    
    # to whether join the room
    def receive_alert(self, req):
        print(f" {self.name} received alert with id {req.id}")
        return True

    def proposal_submission(req, round):
        cost = my_cost_calculation(req.nego_content)
        return {self.name: {"cost": cost}}

    def round_table(req, round, other_proposals):
        # Return ranking, always assume our proposal is the best
        return [self.name]

    def concession(req, round, final_proposals):
        # always accept all proposals
        return True

    def assignment(self, req, proposal: Dict):
        # accept then execute the assignment

agent = Agent(agent_name)
agent.spin()

# ....
# Agent b can send in an alert with fn call
agent_b.submit("task2", {})
```

## Potential Improvements

 - Use [fcl](https://github.com/BerkeleyAutomation/python-fcl/) For path conflict detection
 - Fault tolerance and better state synchronization
 - task swapping? return of evaluation heuristic for better replanning of conflict task
 - more robust `nego_queue` task queue handling.
