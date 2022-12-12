#!/usr/bin/env python3

from typing import Dict, List
from abc import ABC, abstractmethod

from internal_deneg import InternalDeNeg as IDeNeg 
from internal_deneg import InternalEvaluator as IEval
from utils import NegoRequest
from deneg_msgs.msg import Alert

##############################################################################
class Type:
    MISCELLANEOUS: int = Alert.MISCELLANEOUS
    TASK_ALLOCATION: int = Alert.TASK_ALLOCATION
    PATH_RESOLUTION: int = Alert.PATH_RESOLUTION

##############################################################################
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

##############################################################################

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
    def receive_alert(self, req: NegoRequest) -> bool:
        """
        This callback function will be called, to alert agent to
        whether join the negotion room, by return a Bool
        """
        return False

    @abstractmethod
    def proposal_submission(
            self, req: NegoRequest, round: int
        ) -> Dict:
        """
        A callback function for user to submit the proposal.
        This should comply with the evalutation format.
        """
        return {}

    @abstractmethod
    def round_table(
            self, req: NegoRequest, round: int, all_proposals: Dict[str, Dict]
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
            self, req: NegoRequest, round: int, final_proposals: List
        ) -> bool:
        """
        this callback function is called at the very end of the
        negotiation process. User can choose, to accept or reject
        the proposal by returning a bool. If reject, this will
        trigger another round of negotiation.
        """
        return False

    @abstractmethod
    def assignment(self, req: NegoRequest, proposal: Dict):
        """
        This callback function is called when the negotiation
        ended, and the assignment is made.
        """
        # TODO: what is assignment? for path conflict is proposal?
        pass

    def submit(
            self,
            id: str,
            content: Dict,
            type = Type.TASK_ALLOCATION,
            self_join = True
        ):
        """
        Agent can use this fn method to send an alert to other
        agents, which will initiate a nego process
        @id:            unique id for the negotiation
        @content:       content of the negotiation
        @type:          type of the negotiation
        @self_join:     whether to join the negotiation
        """
        self.deneg.submit(id, content, type, self_join)

    def participants(self) -> List[str]:
        """
        return the list of participants in the negotiation room
        """
        return self.deneg.participants()

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
