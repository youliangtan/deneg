# DeNeg

toy problem of decentralized negotiation system

<WORK IN PROGRESS>

This is to create a simple decentralized negotiation library


Dependencies
 - ros2 (Tested on humble)


Compilation
```bash
colcon build
```

## Example

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

    def proposal_submission(round):
        cost = cost_calcualtion(self.nego_content)
        return {self.name: {"cost": cost}}

    def round_table(round, other_proposals):
        # we willl always asusume our 
        name=  io nvn k
        return {self.name: {"cost": cost}}

a = Agent(agent_name)
a.spin()
```
