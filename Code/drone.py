import math
import random
from queue import PriorityQueue

import numpy as np


class Drone:
    """
    This class will encode a singular drone,its attributes and its functionality
    """

    def __init__(self, id, position, sensor, environment_drones, ground_station, environment, drone_deflect_clockwise):
        """
        The constructor for the drone class
        :param id: The id number of the drone
        :param position: Where the drone is in the environment
        :param sensor: The lidar class
        :param environment_drones: A list of the drones in the environment
        :param ground_station: The ground station instance
        :param environment: The environment instance
        :param drone_deflect_clockwise: A boolean to determine if the drone deflects clockwise or anti-clockwise
        """
        self.goal_position = None
        self.id = id
        self.local_environment = []
        self.immediate_environment = []
        self.previous_position = None
        self.current_position = position
        self.sensor = sensor
        self.sensor_data = []
        self.intermediate_node = None
        self.environment_drones = environment_drones
        self.path = []
        self.checked_nodes = []
        self.ground_station = ground_station
        self.env = environment
        self.chunks_to_map = []
        self.mapped_chunks = []
        self.communication_range = 250
        self.deflection_clockwise = drone_deflect_clockwise
        self.set_goal_position()

    def sense_environment(self):
        """
        This function initiates one rotation from the simulated LIDAR sensor onboard the drone and returns the sensed
        obstacles
        :return: A list of sensed positions for the point cloud map
        """

        self.sensor.current_position = self.current_position
        self.data_storage(self.sensor.sense_obstacles(self.current_position))

        self.communicate_to_drone()
        self.move()
        self.checked_nodes = []
        rand = random.randint(0, 10000)
        if rand < 5:
            self.goal_position = (100, 100)

    @staticmethod
    def a_d_2pos(distance, angle, drone_position):
        """
        A function to convert raw data to point on the map
        :param distance: The distance of the point from the drone
        :param angle: The angel from the drone
        :param drone_position: The position of the drone
        :return: the integer location of the point in the environment
        """
        x = distance * math.cos(angle) + drone_position[0]
        y = -distance * math.sin(angle) + drone_position[1]
        return int(x), int(y)

    def data_storage(self, data):
        """
        This function takes the raw sensor data, converts it to points on the map and saves it to the local environment
        and the immediate environment
        :param data: The raw data from the sensor
        :return: None
        """
        if len(self.immediate_environment) > 600:
            self.immediate_environment = self.immediate_environment[-600:]
        if data is not False:
            for element in data:
                point = self.a_d_2pos(element[0], element[1], element[2])
                if point not in self.local_environment:
                    self.local_environment.append(point)
                self.immediate_environment.append(point)

    def move(self):
        """
        This function contains the logic for how to move the drone, deciding when to generate new paths, when the
        goal position has been reached and handling error logic from the path generation
        :return: None
        """

        if len(self.path) == 0:
            found_path = self.generate_path()
            while not found_path:
                found_path = self.generate_path()
        elif self.current_position == self.intermediate_node:
            self.path = []
            found_path = self.generate_path()
            while not found_path:
                found_path = self.generate_path()
        elif self.find_distance_to_point(self.current_position, self.goal_position) < 20:
            if len(self.chunks_to_map) != 0:
                self.mapped_chunks.append(self.chunks_to_map[0])
                self.chunks_to_map.remove(self.chunks_to_map[0])
            self.set_goal_position()
            # print("Setting goal position because within 20 pixels of goal")
            self.path = []
            found_path = self.generate_path()
            while not found_path:
                found_path = self.generate_path()
        else:
            if self.move_too_close_too_object(self.path[1]):
                self.path = []
                found_path = self.generate_path()
                while not found_path:
                    found_path = self.generate_path()
            else:
                self.previous_position = self.current_position
                self.current_position = self.path[1]
                del self.path[0]

    def generate_path(self):
        """
        This function generates a path to the intermediate node using the A* algorithm
        :return: A boolean of False if no path is set and True if one is
        """
        intermediate_node_set = self.set_intermediate_node()
        while not intermediate_node_set:
            # print("Setting new goal position because stuck in corner")
            self.goal_position = (random.randint(self.current_position[0] - 100, self.current_position[0] + 100),
                                  random.randint(self.current_position[1] - 100, self.current_position[1] + 100))
            intermediate_node_set = self.set_intermediate_node()
        frontier = PriorityQueue()
        frontier.put((0, self.current_position))
        came_from = dict()
        cost_so_far = dict()
        came_from[self.current_position] = None
        cost_so_far[self.current_position] = 0

        while not frontier.empty():
            current = frontier.get()[1]

            if current == self.intermediate_node:
                break

            possible_moves = self.generate_possible_moves(current)
            buffered_possible_moves = []
            for move in possible_moves:
                if not self.move_too_close_too_object(move):
                    buffered_possible_moves.append(move)

            for next in buffered_possible_moves:
                new_cost = self.find_distance_to_point(current, next)
                self.checked_nodes.append(next)
                if len(self.checked_nodes) > 1000:
                    self.checked_nodes = []
                    self.goal_position = (
                        random.randint(self.current_position[0] - 100, self.current_position[0] + 100),
                        random.randint(self.current_position[1] - 100, self.current_position[1] + 100))
                    # print("Setting goal position because over 1000 searched nodes")
                    return False
                if next not in came_from:
                    cost_so_far[next] = new_cost
                    priority = self.find_distance_to_point(self.intermediate_node, next)
                    frontier.put((priority, next))
                    came_from[next] = current
                if next is self.goal_position:
                    break

        current = self.intermediate_node
        while current != self.current_position:
            self.path.append(current)
            current = came_from[current]
        self.path.reverse()
        return True

    def set_intermediate_node(self):
        """
        This function set the intermediate node between the current position and the goal position. it does this through
        interpolation and radial deflection.
        :return: A boolean of False if no intermediate node is set and True if one is
        """

        possible_nodes = []
        next_node = None
        count = 0

        if len(possible_nodes) == 0:
            v = (self.goal_position[0] - self.current_position[0], self.goal_position[1] - self.current_position[1])
            v_magnitude = self.find_distance_to_point(self.current_position, self.goal_position)
            u = (v[0] / v_magnitude, v[1] / v_magnitude)

            next_node = (int(self.current_position[0] + (50 * u[0])),
                         int(self.current_position[1] + (50 * u[1])))

            while self.check_if_wall_in_the_way(next_node):
                next_node = self.deflect_node(next_node)
                count += 10
                if count == 190:
                    return False

            self.intermediate_node = next_node
        for i in range(len(possible_nodes)):
            if next_node is not None:
                next_node = possible_nodes[i]
            else:
                if possible_nodes[i][1] > next_node[1]:
                    next_node = possible_nodes[i]
            self.intermediate_node = next_node[0]
        return True

    def check_if_wall_in_the_way(self, next_node):
        """
        A function that takes in a node and checks if there is a wall between the drones current position and the node
        :param next_node: The next node in (x , y) format
        :return: Returns True if there is a wall in the way and False if there is not
        """
        for i in range(10, 100, 5):
            # Interpolation
            u = i / 100
            x = int(next_node[0] * u + self.current_position[0] * (1 - u))
            y = int(next_node[1] * u + self.current_position[1] * (1 - u))
            point = (x, y)
            buffered_points = self.calculate_points_around_a_point(point)
            buffered_points.append(point)
            for point in buffered_points:
                if point == self.current_position:
                    buffered_points.remove(point)
            for point in buffered_points:
                if self.move_too_close_too_object(point):
                    return True
        return False

    def deflect_node(self, next_node):
        """
        A function to deflect the intermediate node by + or - 10 degrees if there is a wall in the way
        :param next_node: The intermediate node in (x , y) format
        :return: The new intermediate node in (x , y) format
        """
        # theta = arctan(y2-y1 / x2-x1)

        deflection = 10
        if not self.deflection_clockwise:
            deflection = -10
        length = 20
        angel = math.atan2(next_node[1] - self.current_position[1],
                           next_node[0] - self.current_position[0]) + math.radians(deflection)
        x = length * math.cos(angel) + self.current_position[0]
        y = length * math.sin(angel) + self.current_position[1]
        return (int(x), int(y))

    def check_immediate_environment(self):
        """
        A function to return the drones immediate environment. If the immediate environment is above 500 points it is
        reduced. This is to improve processing times.
        :return: The immediate environment
        """
        if len(self.immediate_environment) > 500:
            return self.immediate_environment[-500:]
        return self.immediate_environment

    def set_goal_position(self):
        """
        This function sets the drones' goal position based of the chunks still left to map.
        If the chunks to mpa list is empty the goal position becomes the ground station
        :return: None
        """
        if len(self.chunks_to_map) != 0:
            next_chunk = self.chunks_to_map[0]
            self.goal_position = (random.randint(next_chunk[0] + 50, next_chunk[1] - 50),
                                  random.randint(next_chunk[2] + 50, next_chunk[3] - 50))
        else:
            # return to ground Station
            self.goal_position = (100, 100)

    def any_points_within_range(self, position, point, range):
        """
        Checks if a defined point is within range
        :param position: The start position
        :param point: The end position
        :param range: The max distance between
        :return: True if point is within range and False if not
        """
        if self.find_distance_to_point(position, point) < range:
            return True
        else:
            return False

    def move_too_close_too_object(self, point):
        """
        Returns true if the defined point is either inside a wall or within 10 pixels of it
        :param point: The point queried
        :return: Boolean
        """

        for data in self.immediate_environment[-500:]:
            if self.find_distance_to_point(point, data) < 10:
                return True
        return False

    def generate_possible_moves(self, position):
        """
        This returns a list of the possible moves around a point
        :param position: The current position
        :return: The list of possible moves
        """
        """
        This function generates the 8 possible moves around the current position and returns them as a list
        :return: A list of 8 next positions
        """
        possible_moves = self.calculate_points_around_a_point(position)
        return possible_moves

    @staticmethod
    def calculate_points_around_a_point(p):
        """
        A function that calculates the 8 moves around a given point
        :param p: The given point
        :return: The list of points around it
        """
        points = [((p[0] - 1), (p[1] + 1)), ((p[0]), (p[1] + 1)), ((p[0] + 1), (p[1] + 1)),
                  ((p[0] - 1), p[1]), ((p[0] + 1), p[1]),
                  ((p[0] - 1), (p[1] - 1)), ((p[0]), (p[1] - 1)), ((p[0] + 1), (p[1] - 1))]
        return points

    @staticmethod
    def find_distance_to_point(next_position, goal_position):
        """
        Finds the euclidian distance between a next position and the goal node
        :param next_position: One of the next positions from the list
        :param goal_position: The second position
        :return: The euclidian distance
        """
        x1 = next_position[0]
        y1 = next_position[1]
        x2 = goal_position[0]
        y2 = goal_position[1]
        return math.sqrt(math.pow((x2 - x1), 2) + math.pow((y2 - y1), 2))

    def communicate_to_drone(self):
        """
        This function implements the communication protocols between the drones, currently not implemented

        Requires implementation of communications to other drones to update local map
        :return: None
        """
        local_drones = self.check_env_for_drones()
        for drone in local_drones:
            if self.local_environment:
                if self.find_distance_to_point(self.goal_position, drone.current_position) <= 50 and len(
                        self.chunks_to_map) != 0:
                    self.mapped_chunks.append(self.chunks_to_map.pop(0))
                drone.add_data_to_local_env(self.local_environment)
                self.mapped_chunks.append(drone.mapped_chunks)
                self.chunks_to_map = [chunk for chunk in self.chunks_to_map if chunk not in drone.mapped_chunks]

    def check_env_for_drones(self):
        """
        A function to check the environment for any nearby drones to communicate with. This is done by scanning the
        environment and determine the distance to all the drones, if it is within the range communications can be
        established.
        :return: The list of drone instances that are close enough
        """
        local_drones = []
        for drone in self.env.drones:
            if drone.id != self.id:
                if self.find_distance_to_point(self.current_position, drone.current_position) \
                        <= self.communication_range:
                    local_drones.append(drone)
        return local_drones

    def add_data_to_local_env(self, data):
        """
        Adds data local environment data from nearby drones to this drones local environment
        :param data: A list of points in the other drones local environment
        :return: None
        """
        if self.local_environment:
            self_local_np = np.array(self.local_environment)
            other_local_np = np.array(data)
            concat = np.concatenate((self_local_np, other_local_np), axis=0)
            remove_duplicates = np.unique(concat, axis=0)
            self.local_environment = remove_duplicates.tolist()
        else:
            self.local_environment = data

    def set_chunks(self, chunks):
        """
        Used by the ground station to set the chunks that need mapping for the drone
        :param chunks: The list of chunks
        :return: None
        """
        if len(self.chunks_to_map) == 0:
            self.chunks_to_map.extend(chunks)
            self.set_goal_position()
        else:
            self.chunks_to_map.extend(chunks)
