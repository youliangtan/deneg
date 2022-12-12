from utils import State
from deneg import DeNeg, Evaluator
from deneg_msgs.msg import Notify


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

proposals = {
    "agent1": {
        "size": 1.0,
        "path": [
            {"x": 6.0, "y": 4.0, "time": 0.0},
            {"x": 5.0, "y": 4.0, "time": 1.0},
            {"x": 4.0, "y": 4.0, "time": 2.0},
            {"x": 3.0, "y": 4.0, "time": 3.0},
            {"x": 2.0, "y": 4.0, "time": 4.0},
            {"x": 1.0, "y": 4.0, "time": 5.0},
            {"x": 1.0, "y": 3.0, "time": 6.0},
        ]
    },
    "agent2": {
        "size": 1.0,
        "path": [
            {"x": 1.0, "y": 1.0, "time": 4.0},
            {"x": 1.0, "y": 2.0, "time": 5.0},
            {"x": 1.0, "y": 3.0, "time": 6.0},
            {"x": 2.0, "y": 3.0, "time": 7.0},
            {"x": 3.0, "y": 3.0, "time": 8.0},
            {"x": 4.0, "y": 3.0, "time": 9.0},
        ]
    },
    "agent3": {
        "size": 1.0,
        "path": [
            {"x": 5.0, "y": 1.0, "time": 4.0},
            {"x": 4.0, "y": 1.0, "time": 5.0},
            {"x": 4.0, "y": 2.0, "time": 6.0},
            {"x": 3.0, "y": 2.0, "time": 7.0},
            {"x": 3.0, "y": 3.0, "time": 8.0},
            {"x": 3.0, "y": 4.0, "time": 9.0},
            {"x": 3.0, "y": 5.0, "time": 10.0},
        ]
    },
}
res = Evaluator.PathConflictEvaluater(proposals)
assert res == []

# Second round of proposals
proposals["agent2"]["path"][2] = \
    {"x": 1.0, "y": 2.0, "time": 6.0},
res = Evaluator.PathConflictEvaluater(proposals)
assert res == ["agent1"]

# Second round of proposals
proposals["agent2"]["path"][2] = {"x": 1.0, "y": 2.0, "time": 6.0},
res = Evaluator.PathConflictEvaluater(proposals)
assert res == ["agent1"]

# Third round of proposals
proposals["agent2"]["path"][3] = {"x": 4.0, "y": 3.0, "time": 7.0},
proposals["agent2"]["path"][4] = {"x": 4.0, "y": 4.0, "time": 8.0},
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
