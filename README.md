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
python3 deneg/deneg_core/scripts/agent.py
```

## Code Snippet

Simple example of using DeNeg for task allocation 

```py
class Agent(DeNeg):
    def __init__(self, name):
        super().__init__(name)
    
    # to whether join the room
    def receive_alert(self, id, content):
        print(f" {self.name} received alert with id {id}")
        self.nego_content = content
        return True

    def proposal_submission(id, round):
        cost = my_cost_calculation(self.nego_content)
        return {self.name: {"cost": cost}}

    def round_table(id, round, other_proposals):
        # Return ranking, always assume our proposal is the best
        return [self.name]

    def concession(id, round, final_proposals):
        # always accept all proposals
        return True

    def assignment(self, id, assignment: Dict):
        # accept then execute the assignment

agent = Agent(agent_name)
agent.spin()

# ....
# Agent b can send in an alert with fn call
agent_b.send_alert("task2", {})
```
