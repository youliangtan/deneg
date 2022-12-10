#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from deneg_msgs.msg import Alert, Proposal, Notify

import threading
import time
import json

from typing import Dict, List
from abc import ABC, abstractmethod
from utils import random_color, bcolors


internel_spin_mutex = True

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
        print(proposals)
        name_list = []
        cost_list = []
        for name, content in proposals.items():
            name_list.append(name)
            cost_list.append(content['cost'])
        # similar to numpy.argsort
        sorted_cost_idx = \
            [i[0] for i in sorted(enumerate(cost_list), key=lambda x:x[1])]
        sorted_name = [name_list[i] for i in sorted_cost_idx]
        ranking = sorted_name
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

    # @abstractmethod
    def round_table(
            self, round: int, other_proposals: List
        ) -> List:
        """
        This callback will be called in the end of each nego round.
        The round table is where each participant will take in
        proposals of other agents, and evaluate them.

        return:
            provide the ranking of proposal_ids. Omit out the
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
        to return a False, in which to reject this proposal.
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

    def leave(self, id):
        """
        To leave the negotiation room
        """
        self.__leave(id)

    def spin(self):
        """
        blocking function to spin the node
        """
        self.__spin()

    def shutdown(self):
        """
        shut down the deneg
        """
        self.logger("shutdown deneg")
        if rclpy.ok():
            rclpy.shutdown()
    
    def error_callback(self, e):
        self.logger("error_callback: " + str(e))

    # def __del__(self):
    #     self.logger("del deneg")

    #############################################################

    def __internal_init(self, name):
        self.name = name
        self.log_color = random_color()
        self.logger("Create deneg participant")

        if not rclpy.ok():
            rclpy.init()
        super().__init__(name)
        self.alert_pub_ = self.create_publisher(
            Alert, 'nego/alert', 10)
        self.alert_sub_ = self.create_subscription(
            Alert, 'nego/alert', self.__alert_callback, 10)
        self.__notify_pub_ = self.create_publisher(
            Notify, 'nego/notify', 10)
        self.proposal_sub_ = self.create_subscription(
            Notify, 'nego/notify', self.__notify_callback, 10)
        self.__proposal_pub_ = self.create_publisher(
            Proposal, 'nego/propose', 10)
        self.proposal_sub_ = self.create_subscription(
            Proposal, 'nego/propose', self.__proposal_callback, 10)

        self.request_queue = []
        self.room_proposals = {}

        self.ros_spin_th = \
            threading.Thread(target=self.__rosspin_thread, args=())
        self.ros_spin_th.start()


    def __send_alert(self, id, content, self_join):
        msg = Alert(
                requester=self.name,
                alert_id=id,
                content=json.dumps(content),
            )
        # print(f"[{self.name}] sending alert {id}")
        self.logger(f"Sending alert {id}")
        self.alert_pub_.publish(msg)
        # start a nego process
        if self_join:
            self.start_nego_process(id, self.name)

    def __proposal_callback(self, msg):
        self.room_proposals[msg.proponent] = {"cost": msg.cost}

    def __spin(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        pass

    def __leave(self, id):
        pass

    def __alert_callback(self, msg):
        # ignore msg sent by myself
        if msg.requester == self.name:
            return

        self.logger("callback: msg from:" \
              f"[{msg.requester}, {msg.alert_id}]" )
        join = self.receive_alert(
                id= msg.alert_id,
                content=json.dumps(msg.content)
            )
        # if user choose to join the room
        if join:
            # Start a tracking deneg process
            self.start_nego_process(msg.alert_id, msg.requester)
            self.__notify_pub_.publish(
                Notify(
                    source=self.name,
                    type=Notify.JOIN,
                    room_id=msg.alert_id,
                )
            )

    def __notify_callback(self, msg):
        # ignore msg sent by myself
        if msg.source == self.name:
            return

        # TODO check room id
        if msg.type == Notify.JOIN:
            self.room_proposals[msg.source] = {}
            self.logger(
                f"{msg.source} joined the room of {self.room_proposals.keys()}")
        elif msg.type == Notify.READY:
            self.logger(f"{msg.source} is ready {len(self.room_proposals)} = {msg.data}")
            # if inconsistent, throw error and exit

        elif msg.type == Notify.NEGO_ROUND:
            # TODO: this will get called N-1 times, fix this
            self.logger(f"provide proposal for nego {msg.data}")
            proposal = self.proposal_submission(msg.data)
            self.__proposal_pub_.publish(
                Proposal(
                    proponent=self.name,
                    room_id=msg.room_id,
                    cost = proposal["cost"],
                    # content=json.dumps(proposal),
                )
            )


    def __rosspin_thread(self):
        global internel_spin_mutex
        while rclpy.ok():
            if internel_spin_mutex:
                internel_spin_mutex = False
                # print(f"[{self.name}] spin")
                rclpy.spin_once(self, timeout_sec=0.1)
                internel_spin_mutex = True

    # this will call when we would like to start a nego process
    def start_nego_process(self, id, name):
        # TODO: use a state check to ensure states are correct 
        # among all participants

        self.room_proposals = {self.name: {}}
        self.room_proposals[name] = {}
        self.deneg_process_th = \
            threading.Thread(target=self.__deneg_process_thread, args=(id,))
        self.deneg_process_th.start()

    def __deneg_process_thread(self, id):
        # Start the Nego process
        self.logger(f"start nego process {id}")
        time.sleep(2)

        # Form the Room
        self.logger(f"Forming Room {id}: {self.room_proposals.keys()}")
        self.__notify_pub_.publish(
            Notify(
                source=self.name,
                type=Notify.READY,
                room_id=id,
                data=len(self.room_proposals),
            )
        )
        time.sleep(1)
        
        # check if all agents are ready
        
        # start negotiation rounds
        self.logger(f"Start negotiation rounds {id}")
        for i in range(1):
            self.logger(f"Round {i}")
            # seek for proposals
            self.__notify_pub_.publish(
                Notify(
                    source=self.name,
                    type=Notify.NEGO_ROUND,
                    room_id=id,
                    data=i,
                )
            )
            time.sleep(0.5)

            # round table session
            result = self.round_table(i, self.room_proposals)

            # consensus
            # if consensus reached, break
            consent = self.concession(result)

        # TODO

    def logger(self, msg):
        print(f"{self.log_color}[{self.name}] {msg}{bcolors.ENDC}")

def try_spin(node):
    pass
    # rclpy.spin_once(node, timeout_sec=0.1)

    #############################################################
