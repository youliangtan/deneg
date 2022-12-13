#!/usr/bin/env python3

from typing import Dict, List
from abc import ABC, abstractmethod

from deneg.internal_deneg import InternalDeNeg as IDeNeg 
from deneg.internal_deneg import InternalEvaluator as IEval
from deneg.internal_deneg import InternalResultsAggregator as IResAgg
from deneg.utils import NegoRequest
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
class ResultsAggregator:
    def ConflictFreeParticipant(results):
        """
        This will return the conflict free participants
        """
        return IResAgg.ConflictFreeParticipant(results)

    def RankBasedVoting(results):
        """
        Ranking policy for the round table session", vote score is based
        on the ranking sequence, the higher ranked agent will get higher
        score
        """
        return IResAgg.RankBasedVoting(results)

##############################################################################

class DeNeg(ABC):
    def __init__(
            self, name: str, debug: bool = False
        ):
        """
        Init Decentralized Negotiation library
        @name:           need to be unique
        @debug:          enable debug mode
        """
        self.deneg = IDeNeg(
                name, debug, self.receive_alert,
                self.proposal_submission, self.round_table,
                self.concession, self.assignment, self.aggregate_results
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
            self, 
            req: NegoRequest, round: int, all_proposals: Dict[str, Dict]
        ) -> List[str]:
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
            self, req: NegoRequest, round: int, result: List
        ) -> bool:
        """
        this callback function is called at the very end of the
        negotiation process. User can choose, to accept or reject
        the proposal by returning a bool. If reject, this will
        trigger another round of negotiation.
        @result:        names of parcipants which are valid/in rank mode
        """
        return False

    @abstractmethod
    def assignment(self, req: NegoRequest, proposal: Dict):
        """
        This callback function is called when the negotiation
        ended, and the assignment is made.
        """
        pass

    def aggregate_results(self, results: Dict) -> List[str]:
        """
        This callback function is called when the it is time to aggregate
        all evaluation results from all participants. A default method
        is provided here according to Request Type, User can override this
        """
        if self.current_request().type == Alert.PATH_RESOLUTION:
            return ResultsAggregator.ConflictFreeParticipant(results)
        else:
            return ResultsAggregator.RankBasedVoting(results)

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

    def current_request(self) -> NegoRequest:
        """
        return the current request
        """
        return self.deneg.current_request()

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
