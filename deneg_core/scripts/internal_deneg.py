#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from deneg_msgs.msg import Alert, Proposal, Notify

from threading import Thread, Lock
import time
import json

from typing import Dict, List, Callable
from abc import ABC, abstractmethod
from utils import random_color, bcolors
from utils import State, Participant

internel_spin_mutex = Lock()
STATE_DELAY = 0.8
MAX_NEGO_ROUNDS = 5

# #############################################################
class InternalEvaluator:
    def LowestCostEvaluater(proposals):
        name_list = []
        cost_list = []
        for name, content in proposals.items():
            if 'cost' not in content:
                print(f"Warning! [{name}] no cost attr")
                return []

            name_list.append(name)
            cost_list.append(content['cost'])

        # similar to numpy.argsort
        sorted_cost_idx = \
            [i[0] for i in sorted(enumerate(cost_list), key=lambda x:x[1])]
        sorted_names = [name_list[i] for i in sorted_cost_idx]
        return sorted_names

    def LowestTotalCostEvaluater(proposals):
        name_list = []
        cost_list = []
        for name, content in proposals.items():
            if 'cost' not in content or 'current_cost' not in content:
                print(f"Warning! [{name}] no cost/current_cost attr")
                return []

            name_list.append(name)
            cost_list.append(content['cost'] + content['current_cost'])

        # similar to numpy.argsort
        sorted_cost_idx = \
            [i[0] for i in sorted(enumerate(cost_list), key=lambda x:x[1])]
        sorted_names = [name_list[i] for i in sorted_cost_idx]
        return sorted_names

    def PathConflictEvaluater(proposals):
        # TODO: use FCL to check collision
        name_list = proposals.keys()
        non_collision_names = list(proposals.keys())
        for n in name_list:
            if 'path' not in proposals[n]:
                print(f"Warning! [{n}] no path attr")
                return []
            
        for n in name_list:
            for m in name_list:
                if n == m:
                    continue

                # check collision
                p1 = proposals[n]["path"]
                p2 = proposals[m]["path"]
                for i in range(len(p1)):
                    for j in range(len(p2)):
                        if p1[i] == p2[j]:
                            print("collision detected:", p1[i], p2[j],
                                f"between {n} and {m}")
                            if m in non_collision_names:
                                non_collision_names.remove(m)
        return non_collision_names

#############################################################

class InternalDeNeg(Node):
    def __init__(
            self,
            name: str,
            receive_alert : Callable,
            proposal_submission : Callable,
            round_table : Callable,
            concession : Callable,
            assignment : Callable,
        ):
        """
        Init Decentralized Negotiation library
        @name:           need to be unique
        """
        self.debug = True # CONFIG
        self.unit_test = True # CONFIG

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

        # Tracking 
        self.nego_queue = []
        self.nego_parcipants = {}

        self.ros_spin_th = \
            Thread(target=self.__rosspin_thread, args=())
        self.ros_spin_th.start()

        # Register callbacks
        self.receive_alert = receive_alert
        self.proposal_submission = proposal_submission
        self.round_table = round_table
        self.concession = concession
        self.assignment = assignment
    
    def shutdown(self):
        """
        shut down the deneg
        """
        self.logger("shutdown deneg")
        if rclpy.ok():
            rclpy.shutdown()

    #############################################################

    def send_alert(self, id, content, self_join):
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
        self.nego_parcipants[msg.proponent].proposal = json.loads(msg.content)

    def spin(self):
        global internel_spin_mutex
        self.end_spin_thread = True
        while rclpy.ok():
            if not internel_spin_mutex.locked():
                internel_spin_mutex.acquire()
                rclpy.spin_once(self, timeout_sec=0.1)
                internel_spin_mutex.release()

    def leave(self, id):
        self.logger(f"leave {id}")
        self.update_state(Notify.LEAVE, id)
        self.nego_queue.remove(id)

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
            self.update_state(Notify.JOIN)

    def __notify_callback(self, msg):
        # ignore msg sent by myself
        if msg.source == self.name:
            return

        if len(self.nego_queue) == 0:
            self.logger("no nego process is running")
            return

        current_id = self.nego_queue[0]

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
            self.logger(f"{msg.source} is ready")
            assert len(self.nego_parcipants) == msg.data
            # if inconsistent, throw error and exit

        elif msg.type == Notify.CONSENT:
            self.nego_parcipants[msg.source].state = Notify.CONSENT
            self.nego_parcipants[msg.source].data = msg.data
            self.logger(f"{msg.source} is consented {len(self.nego_parcipants)}")

        # Nego state
        elif msg.type == Notify.NEGO:
            # TODO: this will get called N-1 times, fix this
            self.nego_parcipants[msg.source].state = Notify.NEGO
            self.logger(f"provide proposal for nego {msg.data}")
            
            # this ensures that the proposal is not called multiple times 
            # and overwritten
            if self.nego_parcipants[self.name].self_proposal:
                proposal = self.nego_parcipants[self.name].self_proposal
            else:
                proposal = self.proposal_submission(current_id, msg.data)
                self.nego_parcipants[self.name].self_proposal = proposal

            self.__proposal_pub_.publish(
                Proposal(
                    proponent=self.name,
                    room_id=msg.room_id,
                    content=json.dumps(proposal),
                )
            )

        # Rank state
        elif msg.type == Notify.RANK:
            self.nego_parcipants[msg.source].state = Notify.RANK
            self.nego_parcipants[msg.source].ranking = msg.ranking
            self.logger(f"ranking from {msg.source}: {msg.ranking}")

        # leave state
        elif msg.type == Notify.LEAVE:
            self.logger(f"{msg.source} left the room")
            self.nego_parcipants.pop(msg.source)

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

    def check_participants_data(self, target_data=1):
        """check if all participants are consented"""
        for pcpt in self.nego_parcipants.values():
            if pcpt.data != target_data:
                return False
        return True

    def update_state(self, state, data=0, ranking=[]):
        """update the state of the agent and notify others about the state update too"""
        self.nego_parcipants[self.name].state = state
        self.nego_parcipants[self.name].data = data
        self.nego_parcipants[self.name].ranking = ranking
        self.__notify_pub_.publish(
            Notify(
                source=self.name,
                type=state,
                room_id=self.nego_queue[0],
                data=data,
                ranking=ranking,
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
        time.sleep(STATE_DELAY)

        # Form the Room
        self.logger(f"Forming Room {id}: {self.nego_parcipants.keys()}")
        self.update_state(Notify.READY, len(self.nego_parcipants))
        time.sleep(STATE_DELAY)

        # check if all agents are ready
        if not self.check_participants_state(Notify.READY):
            err = f"Error: not all agents are READY, {self.nego_parcipants}"
            if self.unit_test:
                assert False, err
            self.logger(err)
            self.leave(id)
            return

        # sync all agents
        time.sleep(STATE_DELAY)

        # start negotiation rounds
        self.logger(f"Start negotiation rounds {id}")
        for r in range(MAX_NEGO_ROUNDS):
            self.logger(f"Round {r}")
            # seek for proposals
            self.update_state(Notify.NEGO, data=r)
            time.sleep(STATE_DELAY)

            # Assertion: check if all agents are NEGO
            if not self.check_participants_state(Notify.NEGO):
                err = f"Error: not all agents are NEGO, {self.nego_parcipants}"
                if self.unit_test:
                    assert False, err
                self.logger(err)
                self.leave(id)
                return

            time.sleep(STATE_DELAY)

            # round table session
            # this extract all proposals from the nego_parcipants
            proposals_dict = {
                name: participant.proposal
                for name, participant in self.nego_parcipants.items()
            }

            rank = self.round_table(id, r, proposals_dict)
            self.update_state(Notify.RANK, data=r, ranking=rank)
            time.sleep(STATE_DELAY)


            # Assertion: check if all agents are RANK
            if not self.check_participants_state(Notify.RANK):
                err = f"Error: not all agents are RANK, {self.nego_parcipants}"
                if self.unit_test:
                    assert False, err
                self.logger(err)
                self.leave(id)
                return

            time.sleep(STATE_DELAY)

            # Consensus
            # get ranking of all round table results
            rank = self.rank_based_vote()
            # TODO: think of agg_ranking is published to all agents

            # check if consensus is reached
            time.sleep(STATE_DELAY)
            consent = self.concession(id, r, rank)
            self.update_state(Notify.CONSENT, data=(1 if consent else 0))
            time.sleep(STATE_DELAY)

            # Assertion: check if all agents are CONSENT
            if not self.check_participants_state(Notify.CONSENT):
                err = f"Error: not all agents are CONSENT, {self.nego_parcipants}"
                if self.unit_test:
                    assert False, err
                self.logger(err)
                self.leave(id)
                return

            if self.check_participants_data(target_data=1):
                self.logger(f"Consensus reached {id}")

                # TODO: if only one winner is allowed
                winner = rank[0]
                if winner == self.name:
                    self.assignment(id, self.nego_queue[0]) # placeholder
                break
            else:
                self.logger(f"FAILED! Consensus not reached {id}, TRY AGAIN")
                # remove previous self_proposal from all participants, and try again
                for p in self.nego_parcipants.values():
                    p.self_proposal = None
                time.sleep(STATE_DELAY)

        self.nego_queue.pop(0)
        self.logger(f"end nego process {id}")

    def rank_based_vote(self):
        """Ranking policy for the round table session"""
        scores = {
            name: 0.0
            for name, _ in self.nego_parcipants.items()
        }
        # Apply weighted vote ranking from each participant
        for name, participant in self.nego_parcipants.items():
            for score, name in enumerate(reversed(participant.ranking)):
                scores[name] += score

        self.logger(f"ranking scores: {scores}")
        s_ranks = sorted(scores.items(), key=lambda x: x[1], reverse=True)
        self.logger(f"ranking: {s_ranks}")
        return [name for name, _ in s_ranks]

    def logger(self, msg):
        if self.debug:
            print(f"{self.log_color}[{self.name}] {msg}{bcolors.ENDC}")
