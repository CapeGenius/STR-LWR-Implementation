from threading import Thread, Event
from abc import ABC, abstractmethod
import random
import os
import sys
from core.Util import *
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("No environment variable SUMO_HOME!")
import traci
import sumolib

STRAIGHT = "s"
TURN_AROUND = "t"
LEFT = "l"
RIGHT = "r"
SLIGHT_LEFT = "L"
SLIGHT_RIGHT = "R"
import time
from queue import Queue 
from time import sleep, perf_counter

class RouteController(ABC):
    """
    Base class for routing policy

    To implement a scheduling algorithm, implement the make_decisions() method.
    Please use the boilerplate code from the example, and implement your algorithm between
    the 'Your algo...' comments.

    make_decisions takes in a list of vehicles and network information (connection_info).
        Using this data, it should return a dictionary of {vehicle_id: decision}, where "decision"
        is one of the directions defined by SUMO (see constants above). Any scheduling algorithm
        may be injected into the simulation, as long as it is wrapped by the RouteController class
        and implements the make_decisions method.

    :param connection_info: object containing network information, including:
                            - out_going_edges_dict {edge_id: {direction: out_edge}}
                            - edge_length_dict {edge_id: edge_length}
                            - edge_index_dict {edge_index_dict} keep track of edge ids by an index
                            - edge_vehicle_count {edge_id: number of vehicles at edge}
                            - edge_list [edge_id]

    """
    def __init__(self, connection_info: ConnectionInfo):
        self.connection_info = connection_info
        self.direction_choices = [STRAIGHT, TURN_AROUND,  SLIGHT_RIGHT, RIGHT, SLIGHT_LEFT, LEFT]

    def compute_local_target(self, decision_list, vehicle):
        current_target_edge = vehicle.current_edge
        try:
            path_length = 0
            i = 0

            #the while is used to make sure the vehicle will not assume it arrives the destination bescuse the target edge is too short.
            while path_length <= max(vehicle.current_speed, 20):
                if current_target_edge == vehicle.destination:
                    break
                if i >= len(decision_list):
                    raise UserWarning(
                        "Not enough decisions provided to compute valid local target. TRACI will remove vehicle."
                    )

                choice = decision_list[i]
                if choice not in self.connection_info.outgoing_edges_dict[current_target_edge]:
                    raise UserWarning(
                            "Invalid direction. TRACI will remove vehicle."
                        )
                current_target_edge = self.connection_info.outgoing_edges_dict[current_target_edge][choice]
                path_length += self.connection_info.edge_length_dict[current_target_edge]

                if i > 0:
                    if decision_list[i - 1] == decision_list[i] and decision_list[i] == 't':
                        # stuck in a turnaround loop, let TRACI remove vehicle
                        return current_target_edge

                i += 1

        except UserWarning as warning:
            print(warning)

        return current_target_edge


    @abstractmethod
    def make_decisions(self, vehicles, connection_info):
        pass


class LWRController(RouteController):
    """
    My implementation of the Lighthill-Whitham-Richards model treats a certain number of cars within a network like an
    interval. Through this, without requiring sensors in the network, the LWRController uses the PDE Conservation Law to
    determine whether there is a traffic jam/slowdown and thus makes decisions based on those observations.  
    Example class for a custom scheduling algorithm.
    Utilizes a random decision policy until vehicle destination is within reach,
    then targets the vehicle destination.
    """
    congested_edge = () #global data structure that will be accessed between threads
    congestion_event = threading.Event()

    # """ rely on previous vehicle info from for in loop to calculate  speed/velocity"""


    def __init__(self, connection_info):
        super().__init__(connection_info)

    def make_decisions(self, vehicles, connection_info):
        
        """
        A custom scheduling algorithm can be written in between the 'Your algo...' comments.
        -For each car in the vehicle batch, your algorithm should provide a list of future decisions.
        -Sometimes short paths result in the vehicle reaching its local TRACI destination before reaching its
         true global destination. In order to counteract this, ask for a list of decisions rather than just one.
        -This list of decisions is sent to a function that returns the 'closest viable target' edge
          reachable by the decisions - it is not the case that all decisions will always be consumed.
          As soon as there is enough distance between the current edge and the target edge, the compute_target_edge
          function will return.
        -The 'closest viable edge' is a local target that is used by TRACI to control vehicles
        -The closest viable edge should always be far enough away to ensure that the vehicle is not removed
          from the simulation by TRACI before the vehicle reaches its true destination

        :param vehicles: list of vehicles to make routing decisions for
        :param connection_info: object containing network information
        :return: local_targets: {vehicle_id, target_edge}, where target_edge is a local target to send to TRACI
        """

        

        local_targets = {}

        for vehicle in vehicles:
            '''
            Your algo starts here
            '''
            congested_edge_queue = Queue()
            decision_list = []
            
            # create a list of maps
            unvisited = {edge: 1000000000 for edge in self.connection_info.edge_list} # map of unvisited edges
            visited = {} # map of visited edges

            current_edge = vehicle.current_edge

            #setting up djikstra stuff
            current_distance = self.connection_info.edge_length_dict[current_edge]
            unvisited[current_edge] = current_distance
            path_lists = {edge: [] for edge in self.connection_info.edge_list} #stores shortest path to each edge using directions

            
            # begin multithread operations and allow for communication between both threads
            t1 = Thread(target = djikstra_thread, args=(self, vehicle, visited, unvisited, path_lists, current_edge,congested_edge_queue,))
            t2 = Thread(target = lwr_thread, args=(self, vehicle,congested_edge_queue,))

            t1.start()
            t2.start()

            t1.join()
            t2.join()

            '''
            Your algo ends here
            '''
            local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)

        return local_targets

    def calculate_density_flow(vehicle):

        #calculate speed
        follower_speed = vehicle.follow_speed
        leader_ID = vehicle.getLeader(vehicle.vehicle_ID, dist=0.0)
        leader_speed = getSpeed(leader_ID)

        #change in speed
        del_speed  = follower_speed - leader_speed

        # calculate the density of cars on a certain edge in a given mile
        edge_length = vehicle.connection_info.edge_length_dict[current_edge]
        vehicle_count = vehicle.connection_info.edge_vehicle_count[current_edge] - 1
        density = vehicle_count / edge_length

        flow_rate = density * del_speed

        return flow_rate, density

    # djikstra policy thread
    def djikstra_thread(self, vehicle, visited, unvisited, path_lists, current_edge, congested_edge_queue):
        global congested_edge
        
        while True:
                
                if current_edge not in self.connection_info.outgoing_edges_dict.keys():
                    continue

                if (congestion_event.isSet()):
                    vehicle.reroute()

                for direction, outgoing_edge in self.connection_info.outgoing_edges_dict[current_edge].items():
                    if outgoing_edge not in unvisited:
                        continue
                    edge_length = self.connection_info.edge_length_dict[outgoing_edge]
                    new_distance = current_distance + edge_length
                    if new_distance < unvisited[outgoing_edge]:
                        unvisited[outgoing_edge] = new_distance
                        current_path = copy.deepcopy(path_lists[current_edge])
                        current_path.append(direction)
                        path_lists[outgoing_edge] = copy.deepcopy(current_path)
                        #print("{} + {} : {} + {}".format(path_lists[current_edge], direction, path_edge_lists[current_edge], outgoing_edge))

                
                
                visited[current_edge] = current_distance
                
                
                del unvisited[current_edge]
                
                
                if not unvisited:
                    break
                if current_edge==vehicle.destination:
                    break
                possible_edges = [edge for edge in unvisited.items() if edge[1]]
                current_edge, current_distance = sorted(possible_edges, key=lambda x: x[1])[0]

        return None

    def diff_density_flow(self, vehicle, congested_edge_queue):
        
        # calculate the flow rates and density rates between two time intervals
        flow_rate_one, density_one = self.calculate_density_flow(vehicle)
        time.sleep(0.5)
        flow_rate_two, density_two = self.calculate_density_flow(vehicle)

        # calculate the rate of change
        del_flow_rate = flow_rate_one - flow_rate_two
        del_density = density_one - density_two

        # calculate the congestion derivative
        congestion_derivative = del_flow_rate / del_density

        return congestion_derivative

    # LWR thread to determine congestion in any edge and reroute a car based on traffic congestion
    def lwr_thread(self, vehicle, congested_edge_queue):
        congestion_derivative

        # a while loop that continues until the vehicle reaches its destination
        while (vehicle.destination != vehicle.target_edge):

            # checks to see if the vehicle is in the same edge as prior --> checks the congestion of the new edge
            if (current_edge != vehicle.current_edge):
                current_edge = vehicle.current_edge
                time.sleep(2)
                congestion_derivative = diff_density_flow(self, vehicle)

            # checks the congestion of the current edge
            elif (current_edge != vehicle.current_edge):
                congestion_derivative = diff_density_flow(self, vehicle)

            if (congestion_derivative < 0):
                congested_edge_queue.put(vehicle.current_edge)
                congestion_event.set()

        return None