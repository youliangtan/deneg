import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from deneg_msgs.msg import Alert, Proposal

import threading
import time
import json

from typing import Dict, List
from abc import ABC, abstractmethod

#############################################################
# Not in used
class Participation:
    types = ['cleaning', 'path_conflict']
    max_wait = 1 # secs
    max_rounds = 5
    overall_timeout = 10 # secs

#############################################################
class Evaluator:
    def LowestCostEvaluater(proposals):
        ranking = []
        return ranking

    def PathConflictEvaluater(proposals):
        ranking = []
        return ranking

#############################################################
class DeNeg(Node, ABC):
    def __init__(
            self, name: str
        ):
        """
        Init Decentralized Negotiation library
        @name:           need to be unique
        """
        print(f"create deneg participate : {name}")
        self.__internal_init(name)


    @abstractmethod
    def receive_alert(self, id, content) -> bool:
        """
        This callback function will be called, to alert agent to
        whether join the negotion room, by return a Bool
        """
        return False

    @abstractmethod
    def proposal_submission(
            self, round: int
        ) -> Dict:
        """
        A callback function for user to submit the proposal.
        This should comply with the evalutation format.
        """
        return {}

    @abstractmethod
    def round_table(
            self, round: int, other_proposals: List
        ) -> List:
        """
        This callback will be called in the end of each nego round.
        The round table is where each participant will take in
        proposals of other agents, and evaluate them.

        return:
            return the ranking of proposals that is valid. Omit
            proposals that are not valid.
        """
        # default impl
        ranking = Evaluator.LowestCostEvaluater(other_proposals)
        return ranking

    @abstractmethod
    def concession(
            self, final_proposals: List
        ) -> bool:
        """
        this callback function is called at the very end of the
        negotiation process. Agent will listen to this, and suppose
        to act according to the final proposals. User can choose
        to return a False, in which to decline this proposal.
        """
        return False

    def send_alert(
            self, id, content: Dict, self_join = True
        ):
        """
        Agent can use this fn method to send an alert to other
        agents, which will initiate a nego process
        """
        self.__send_alert(id, content, self_join)

    def exit(self, id):
        """
        To exit the negotiation room
        """
        self.__exit(id)

    def spin(self):
        """
        blocking function to spin the node
        """
        self.__spin()


    #############################################################

    def __internal_init(self, name):
        self.name = name
        new_node = False
        if not rclpy.ok():
            new_node = True
            rclpy.init()
        super().__init__(name)
        self.alert_pub_ = self.create_publisher(
            Alert, 'nego/alert', 10)
        self.alert_sub_ = self.create_subscription(
            Alert, 'nego/alert', self.__alert_callback, 10)
        self.join_pub_ = self.create_publisher(
            String, 'nego/join', 10)
        self.join_sub_ = self.create_subscription(
            String, 'nego/join', self.__join_callback, 10)

        self.request_queue = []
        self.participant_proposals = {}
        if new_node:
            self.ros_spin_th = \
                threading.Thread(target=self.__rosspin_thread, args=())
            self.ros_spin_th.start()

    def __send_alert(self, id, content, self_join):
        msg = Alert(
                requester=self.name,
                alert_id=id,
                content=json.dumps(content),
            )
        self.alert_pub_.publish(msg)
        if self_join:
            self.participant_proposals[self.name] = {}

    def __spin(self):
        pass

    def __exit(self, id):
        pass

    def __alert_callback(self, msg):
        # ignore msg sent by myself
        if msg.requester == self.name:
            return
        join = self.receive_alert(
                id= msg.alert_id,
                content=json.dumps(msg.content)
            )
        # if user choose to join the room
        if join:
            self.join_pub_.publish(
                String(data=self.name)
            )

    def __join_callback(self, msg):
        # ignore msg sent by myself
        if msg.data == self.name:
            return
        print(f"{msg.data} joined the room!")
        self.participant_proposals[msg.data] = {}

    def __rosspin_thread(self):
        # print('start')
        while rclpy.ok():
            # print('spin')
            rclpy.spin_once(self, timeout_sec=0.1)
            # print('done spin')

    # this will call when max_time is reached.
    def __start_nego_phase(self):
        pass

    #############################################################
