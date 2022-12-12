#!/usr/bin/env python3

from typing import Dict, List
from abc import ABC, abstractmethod

from internal_deneg import InternalDeNeg as IDeNeg 
from internal_deneg import InternalEvaluator as IEval

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
        @return:    ranking list of participant names
        """
        return IEval.LowestCostEvaluater(proposals)

    def LowestTotalCostEvaluater(proposals):
        """
        Rank the total cost proposals from Lowest to Highest
        @return:    ranking list of participant names
        """
        return IEval.LowestTotalCostEvaluater(proposals)

    def PathConflictEvaluater(proposals):
        """
        Indentify non-conflict path proposals
        @return:    participants that are not in conflict
        """
        return IEval.PathConflictEvaluater(proposals)

#############################################################

class DeNeg(ABC):
    def __init__(
            self, name: str
        ):
        """
        Init Decentralized Negotiation library
        @name:           need to be unique
        """
        self.deneg = IDeNeg(
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
            self, id: str, round: int, all_proposals: Dict[str, Dict]
        ) -> List:
        """
        This callback will be called in the end of each nego round.
        The round table is where each participant will take in
        proposals of other agents, and evaluate them.
        @all_proposals:     a dict with key as agent name, and
                            value as the proposal
        @return:
            provide the ranking of proposal_ids. Omit out the
            proposals that are not valid.
        """
        return []

    @abstractmethod
    def concession(
            self, id: str, round: int, final_proposals: List
        ) -> bool:
        """
        this callback function is called at the very end of the
        negotiation process. Agent will listen to this, and suppose
        to act according to the final proposals. User can choose
        to return a False, in which to reject this proposal.
        """
        return False

    @abstractmethod
    def assignment(self, id: str, assignment: Dict):
        pass

    def send_alert(
            self, id: str, content: Dict, self_join = True
        ):
        """
        Agent can use this fn method to send an alert to other
        agents, which will initiate a nego process
        """
        self.deneg.send_alert(id, content, self_join)

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
