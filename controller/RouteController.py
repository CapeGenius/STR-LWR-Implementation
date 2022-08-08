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
    congested_edge = []
    

    # def make_decisions(self, vehicles, connection_info):

    #     local_targets = {}
        
    #     for vehicle in vehicles:
    #         start_edge = vehicle.current_edge



    #     return 0

    # # calculate the change in flow rate and density with a car acting as an interval 
    # def del_density_flow(vehicle):


    #     calculate_density_flow(self, vehicle)

        

    #     return 0
    
    
    # def calculate_density_flow(self, vehicle):
        
    #     # calculate the speed of the car behind
    #     follow_ID, follow_distance = vehicle.getFollower(self,vehicle, dist=0.0)
    #     follow_speed = vehicle.getFollowSpeed(self, follow_ID,)
    #     # calculate the speed of the car in front

    #     # calculate the density of cars on a certain edge in a given mile
    #     edge_length = self.connection_info.edge_length_dict[current_edge]
    #     vehicle_count = self.connection_info.edge_vehicle_count[current_edge] - 1

    #     density = vehicle_count / edge_length
    #     # if needed, finded the length of the edge

    #     # first, calculate the del speed between vehicles and multiply by the k value

    #     # wait another second or millisecond --> do the following again

    #     return density_one, density_two, flow_rate_one, flow_rate_two

    #     # more possible ideas 
    #     # """ rely on previous vehicle info from for in loop to calculate  speed/velocity"""


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
            decision_list = []

            congested_signal = False
            



            i = 0
            while i < 10:  # choose the number of decisions to make in advanced; depends on the algorithm and network
                choice = self.direction_choices[random.randint(0, 5)]  # 6 choices available in total

                # dead end
                if len(self.connection_info.outgoing_edges_dict[start_edge].keys()) == 0:
                    break

                # make sure to check if it's a valid edge
                if choice in self.connection_info.outgoing_edges_dict[start_edge].keys():
                    decision_list.append(choice)
                    start_edge = self.connection_info.outgoing_edges_dict[start_edge][choice]

                    if i > 0:
                        if decision_list[i-1] == decision_list[i] and decision_list[i] == 't':
                            # stuck in a turnaround loop, let TRACI remove vehicle
                            break

                    i += 1

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

    def del_density_flow(flow_rate_one, density_one, flow_rate_two, density_two):
        del_flow_rate = flow_rate_two - flow_rate_one 
        del_density = density_two - density_one

        return del_flow_rate, del_density

    