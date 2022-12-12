#!/usr/bin/env python3

def path_conflicts_proposals(round=0):

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
    if round == 1:
        proposals["agent2"]["path"][2] = {"x": 1.0, "y": 2.0, "time": 6.0}
    elif round == 2:
        proposals["agent2"]["path"][2] = {"x": 1.0, "y": 2.0, "time": 6.0}
        proposals["agent3"]["path"][3] = {"x": 4.0, "y": 3.0, "time": 7.0}
        proposals["agent3"]["path"][4] = {"x": 4.0, "y": 4.0, "time": 8.0}
    return proposals
