from utils import State
from deneg.deneg import DeNeg, Evaluator
from deneg_msgs.msg import Notify

from deneg.examples import path_conflicts_proposals

proposals = {
    "agent1": {"cost": 1.0, "current_cost": 5.1},
    "agent2": {"cost": 2.0, "current_cost": 2.0},
    "agent3": {"cost": 3.0, "current_cost": 4.1},
    "agent4": {"cost": 4.0, "current_cost": 1.8}
}

rank = Evaluator.LowestCostEvaluater(proposals)
assert rank == ["agent1", "agent2", "agent3", "agent4"]

rank = Evaluator.LowestTotalCostEvaluater(proposals)
assert rank == ["agent2", "agent4", "agent1", "agent3"]

#######################################################
# Test Collision

# First round of proposals
proposals = path_conflicts_proposals()
res = Evaluator.PathConflictEvaluater(proposals)
assert res == []

print('----')
# Second round of proposals
proposals = path_conflicts_proposals(round=1)
res = Evaluator.PathConflictEvaluater(proposals)
assert res == ["agent1"]

# Third round of proposals
proposals = path_conflicts_proposals(round=2)
res = Evaluator.PathConflictEvaluater(proposals)

assert "agent1" in res
assert "agent2" in res
assert "agent3" in res

#######################################################

s = State()
assert s.current() == Notify.JOIN
s.next()
assert s.current() == Notify.READY
s.next()
s.next()
assert s.current() == Notify.RANK

# throw bad states
s.leave()
assert s.current() == Notify.LEAVE
s.next()
assert s.current() == Notify.LEAVE
s.error()
assert s.current() == Notify.ERROR

# From start to end
s.reset()
assert s.current() == Notify.JOIN
s.next()
s.next()
s.next()
s.next()
assert s.current() == Notify.CONSENT
s.next()
assert s.current() == Notify.ASSIGNMENT
s.next()
assert s.current() == Notify.ASSIGNMENT

print("all passed")
