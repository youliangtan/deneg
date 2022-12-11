#!/usr/bin/env python3

from typing import Dict, List
from abc import ABC, abstractmethod

from internal_deneg import InternalDeNeg, InternalEvaluator

#############################################################
# Not in used
class Participation:
    types = ['cleaning', 'path_conflict']
    max_wait = 1 # secs
    max_rounds = 5
    overall_timeout = 10 # secs

class Type:
    TaskAssignent = 0
    PathResolution = 1

# class NegoSpecs:
    # type: Type
    # rank_algo: 

#############################################################
class Evaluator:
    def LowestCostEvaluater(proposals):
        """
        Rank the cost proposals from Lowest to Highest
        """
        return InternalEvaluator.LowestCostEvaluater(proposals)

    def PathConflictEvaluater(proposals):
        ranking = []
        return ranking

#############################################################

class DeNeg(ABC):
    def __init__(
            self, name: str
        ):
        """
        Init Decentralized Negotiation library
        @name:           need to be unique
        """
        self.deneg = InternalDeNeg(
                name, self.receive_alert,
                self.proposal_submission, self.round_table,
                self.concession, self.assignment,
            )

    @abstractmethod
    def receive_alert(self, id: str, content) -> bool:
        """
        This callback function will be called, to alert agent to
        whether join the negotion room, by return a Bool
        """
        return False

    @abstractmethod
    def proposal_submission(
            self, id: str, round: int
        ) -> Dict:
        """
        A callback function for user to submit the proposal.
        This should comply with the evalutation format.
        """
        return {}

    @abstractmethod
    def round_table(
            self, id: str, round: int, other_proposals: List
        ) -> List:
        """
        This callback will be called in the end of each nego round.
        The round table is where each participant will take in
        proposals of other agents, and evaluate them.

        return:
            provide the ranking of proposal_ids. Omit out the
            proposals that are not valid.
        """
        return []

    @abstractmethod
    def concession(
            self, id: str, final_proposals: List
        ) -> bool:
        """
        this callback function is called at the very end of the
        negotiation process. Agent will listen to this, and suppose
        to act according to the final proposals. User can choose
        to return a False, in which to reject this proposal.
        """
        return False

    def send_alert(
            self, id: str, content: Dict, self_join = True
        ):
        """
        Agent can use this fn method to send an alert to other
        agents, which will initiate a nego process
        """
        self.deneg.send_alert(id, content, self_join)

    @abstractmethod
    def assignment(self, id: str, assignment: Dict):
        pass

    def leave(self, id):
        """
        To leave the negotiation room
        """
        self.deneg.leave(id)

    def spin(self):
        """
        blocking function to spin the node
        """
        self.deneg.spin()

    def shutdown(self):
        """
        shut down the deneg
        """
        self.deneg.shutdown()
