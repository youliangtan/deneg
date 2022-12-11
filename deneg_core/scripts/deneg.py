#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from deneg_msgs.msg import Alert, Proposal, Notify

from threading import Thread, Lock
import time
import json

from typing import Dict, List
from abc import ABC, abstractmethod
from utils import random_color, bcolors
from utils import State, Participant

internel_spin_mutex = Lock()

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
        name_list = []
        cost_list = []
        for name, content in proposals.items():
            name_list.append(name)
            if 'cost' not in content:
                print(f"Warning! [{name}] no cost attr")
                return []

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
    def receive_alert(self, id: str, content) -> bool:
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
            self, id: str, content: Dict, self_join = True
        ):
        """
        Agent can use this fn method to send an alert to other
        agents, which will initiate a nego process
        """
        self.__send_alert(id, content, self_join)

    @abstractmethod
    def assignment(self, assignment: Dict):
        pass

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

        # Thread handler
        self.end_spin_thread = False

        self.nego_queue = []

        # Tracking 
        self.nego_parcipants = {}
        self.pcpt_states = {}

        self.ros_spin_th = \
            Thread(target=self.__rosspin_thread, args=())
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
        if msg.proponent not in self.nego_parcipants:
            print(f"ERROR! {msg.proponent} doesnt exist during proposal callback")
            # TODO: throw error
        self.nego_parcipants[msg.proponent].proposal = {"cost": msg.cost}

    def __spin(self):
        global internel_spin_mutex
        self.end_spin_thread = True
        while rclpy.ok():
            if not internel_spin_mutex.locked():
                internel_spin_mutex.acquire()
                rclpy.spin_once(self, timeout_sec=0.1)
                internel_spin_mutex.release()

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
            self.nego_parcipants[msg.source] = Participant(name=msg.source)
            self.logger(
                f"{msg.source} joined the room of {self.nego_parcipants.keys()}")

        # Ready state
        elif msg.type == Notify.READY:
            if msg.source not in self.nego_parcipants:
                self.logger(f"{msg.source} is not in the room")
                self.logger(f"ERROR! {msg.proponent} doesnt exist in ready state")
            self.nego_parcipants[msg.source].state = Notify.READY
            self.logger(f"{msg.source} is ready {len(self.nego_parcipants)} = {msg.data}")
            # if inconsistent, throw error and exit

        # Nego state
        elif msg.type == Notify.NEGO:
            # TODO: this will get called N-1 times, fix this
            self.nego_parcipants[msg.source].state = Notify.NEGO
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

        # Rank state
        elif msg.type == Notify.RANK:
            self.nego_parcipants[msg.source].state = Notify.RANK
            self.logger(f"receive rank {msg.data}")
            self.receive_rank(msg.data)

        # leave state
        elif msg.type == Notify.LEAVE:
            self.logger(f"{msg.source} left the room")
            self.nego_parcipants.pop(msg.source)
            self.pcpt_states.pop(msg.source)


    def __rosspin_thread(self):
        global internel_spin_mutex
        while rclpy.ok():
            if self.end_spin_thread:
                break
            if not internel_spin_mutex.locked():
                internel_spin_mutex.acquire()
                rclpy.spin_once(self, timeout_sec=0.1)
                internel_spin_mutex.release()


    def check_participants_state(self, target_state):
        """check if all participants are in the target state"""
        for pcpt in self.nego_parcipants.values():
            if pcpt.state != target_state:
                return False
        return True

    def update_state(self, state, data=0):
        """update the state of the agent and notify others about the state update too"""
        self.nego_parcipants[self.name].state = state
        self.__notify_pub_.publish(
            Notify(
                source=self.name,
                type=state,
                room_id=self.nego_queue[0],
                data=data,
            )
        )

    # this will call when we would like to start a nego process
    def start_nego_process(self, id, name):
        # TODO: use a state check to ensure states are correct 
        # among all participants
        self.nego_queue.append(id) # TODO: expandable queue
        self.nego_parcipants = {self.name: Participant(name=self.name)}
        self.nego_parcipants[name] = Participant(name=name)
        self.deneg_process_th = \
            Thread(target=self.__deneg_process_thread, args=(id,))
        self.deneg_process_th.start()


    def __deneg_process_thread(self, id):
        # Start the Nego process
        self.logger(f"start nego process {id}")
        time.sleep(2)

        # Form the Room
        self.logger(f"Forming Room {id}: {self.nego_parcipants.keys()}")
        self.update_state(Notify.READY, len(self.nego_parcipants))
        time.sleep(1)

        # check if all agents are ready
        assert self.check_participants_state(Notify.READY), \
            f"Error: not all agents are ready, {self.nego_parcipants}"

        # sync all agents
        time.sleep(0.5)

        # start negotiation rounds
        self.logger(f"Start negotiation rounds {id}")
        for i in range(1):
            self.logger(f"Round {i}")
            # seek for proposals
            self.update_state(Notify.NEGO, data=i)
            time.sleep(0.8)
            
            assert self.check_participants_state(Notify.NEGO)

            # round table session
            # this extract all proposals from the nego_parcipants
            proposals_dict = {
                name: participant.proposal
                for name, participant in self.nego_parcipants.items()
            }
            print(proposals_dict)
            result = self.round_table(i, proposals_dict)

            # consensus
            # get ranking of all round table results

            # check if consensus is reached
            consent = self.concession(result)

            # send assignment according to the result
            if consent:
                self.assignment("DONE") # placeholder

        # TODO

    def state_transition(self, state):
        self.pcpt_states[self.name] = state
        pass

    def logger(self, msg):
        print(f"{self.log_color}[{self.name}] {msg}{bcolors.ENDC}")
