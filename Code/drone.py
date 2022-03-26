import math
import random
import numpy as np
from queue import PriorityQueue


class Drone:
    """
    This class will encode a singular drone,its attributes and its functionality
    """

    def __init__(self, id, position, sensor, environment_drones, ground_station):
        """
        The constructor for the drone class
        :param id: The id number of the drone
        :param position: Where the drone is in the environment
        :param sensor: The lidar class
        """
        self.id = id
        self.local_environment = []
        self.previous_position = None
        self.current_position = position
        self.sensor = sensor
        self.sensor_data = []
        self.goal_position = (random.randint(0, 1200), random.randint(0, 600))
        self.intermediate_node = None
        self.environment_drones = environment_drones
        self.path = []
        self.checked_nodes = []
        self.ground_station = ground_station

    def sense_environment(self):
        """
        This function initiates one rotation from the simulated LIDAR sensor onboard the drone and returns the sensed
        obstacles
        :return: A list of sensed positions for the point cloud map
        """

        self.sensor.current_position = self.current_position
        self.data_storage(self.sensor.sense_obstacles(self.current_position))

        self.communicate_to_drone()
        self.move(self.local_environment[:50:-1])
        self.communicate_to_ground_station()
        self.checked_nodes = []

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
        This function takes the raw sensor data, converts it to points on the map and saves i to the local environment
        :param data: The raw data from the sensor
        :return: None
        """
        if data is not False:
            for element in data:
                point = self.a_d_2pos(element[0], element[1], element[2])
                if point not in self.local_environment:
                    self.local_environment.append(point)

    def move(self, recent_data):
        """
        This function moves the drone, currently implements this movement as an A* algorithm, however does not avoid
        obsticals and does not reassign goal nodes
        :return: None
        """
        if len(self.path) == 0:
            self.generate_path(recent_data)
        elif self.current_position == self.intermediate_node:
            self.path = []
            self.generate_path(recent_data)
        elif self.find_distance_to_point(self.current_position, self.goal_position) < 100:
            self.goal_position = (random.randint(0, 1200), random.randint(0, 600))
            self.path = []
            self.generate_path(recent_data)
        else:
            if self.move_too_close_too_object(self.path[1], recent_data):
                self.path = []
                self.generate_path(recent_data)
            else:
                self.previous_position = self.current_position
                self.current_position = self.path[1]
                del self.path[0]

        # for i in range(len(path)):
        #     if i == 0:
        #         possible_moves = self.generate_possible_moves(self.current_position)
        #         current_position_moves = self.generate_possible_moves(self.current_position)
        #     else:
        #         possible_moves = self.generate_possible_moves(path[i-1])
        #     possible_moves = [x for x in possible_moves if x not in path]
        #     for move in possible_moves:
        #         if self.move_too_close_too_object(move, recent_data):
        #             buffered_possible_moves.append(move)
        #
        #     for move in buffered_possible_moves:
        #         distance = self.find_distance_to_point(move, self.goal_position)
        #         if distance < shortest_distance:
        #             shortest_distance = distance
        #             path[i] = move
        # shortest_distance = 10000000
        # if self.previous_position == path[i]:
        #     self.goal_position = (random.randint(0, 1200), random.randint(0, 600))
        #     print("Redirecting")
        # #print(path)
        # for move in current_position_moves:
        #     distance = self.find_distance_to_point(move, path[-1])
        #     if distance < shortest_distance:
        #         shortest_distance = distance
        #         next_move = move
        # if self.current_position == self.goal_position:
        #     self.previous_position = self.current_position
        #     self.current_position = next_move
        #     print("Hit goal")
        #     self.goal_position = (random.randint(0, 1200), random.randint(0, 600))
        # else:
        #     self.previous_position = self.current_position
        #     self.current_position = next_move

    def generate_path(self, recent_data):
        self.set_intermediate_node()
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
                if not self.move_too_close_too_object(move, recent_data):
                    buffered_possible_moves.append(move)

            for next in buffered_possible_moves:
                #new_cost = cost_so_far[current] + self.find_distance_to_point(current, next)
                new_cost = self.find_distance_to_point(current, next)
                self.checked_nodes.append(next)
                if next not in came_from:
                    cost_so_far[next] = new_cost
                    priority = self.find_distance_to_point(self.intermediate_node, next)
                    frontier.put((priority, next))
                    came_from[next] = current
                if next is self.goal_position:
                    break
            if next is self.goal_position:
                break

        current = self.intermediate_node
        while current != self.current_position:
            self.path.append(current)
            current = came_from[current]
        self.path.reverse()

        # if len(self.path) == 0:
        #     self.path.append(self.current_position)
        # while self.path[-1] != self.goal_position:
        #     shortest_distance = 1000000
        #     next_move = (0, 0)
        #     buffered_possible_moves = []
        #     possible_moves = self.generate_possible_moves(self.path[-1])
        #     possible_moves = [x for x in possible_moves if x not in self.path]
        #     for move in possible_moves:
        #         if not self.move_too_close_too_object(move, recent_data):
        #             buffered_possible_moves.append(move)
        #     for move in buffered_possible_moves:
        #         if move != self.goal_position:
        #             # Current next move to end goal distance huristic
        #             distance = self.find_distance_to_point(move, self.goal_position) \
        #                        # + self.find_distance_to_point(move, self.current_position)
        #             if distance <= shortest_distance:
        #                 shortest_distance = distance
        #                 next_move = move
        #         else:
        #             self.path.append(move)
        #             break
        #     if move != self.goal_position:
        #         self.path.append(next_move)
        #         #print(next_move)
        #         #print("Goal" + str(self.goal_position))

    def set_intermediate_node(self):

        possible_nodes = []
        x1, y1 = self.current_position[0], self.current_position[1]
        for angle in np.linspace(0, 2 * math.pi, 50, False):
            x2, y2 = (x1 + 200 * math.cos(angle), y1 - 200 * math.sin(angle))
            for i in range(0, 100):
                # Interpolation
                u = i / 100
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                # If within the window
                if 0 < x < 600 and 0 < y < 1200:
                    # integer_colour = int(self.local_environment[x][y])
                    # colour = (integer_colour & 255, (integer_colour >> 8) & 255, (integer_colour >> 16) & 255)
                    # colour = self.map.get_at((x, y))
                    # if (colour[0], colour[1], colour[2]) == (0, 0, 0):
                    if self.move_too_close_too_object((x,y) ,self.local_environment):
                        # output = [(x,y), math.pow(self.find_distance_to_point(self.current_position, (x,y)) + (100000 - self.find_distance_to_point(self.goal_position, (x,y))),2)]
                        # Store the measurement
                        # possible_nodes.append(output)
                        break
                    elif i == 100:
                        output = [(x, y), math.pow(self.find_distance_to_point(self.current_position, (x, y)) + (100000 - self.find_distance_to_point(self.goal_position, (x, y))),2)]
                        possible_nodes.append(output)

        next_node = None
        if len(possible_nodes) == 0:
            v = (self.goal_position[0] - self.current_position[0], self.goal_position[1] - self.current_position[1])
            v_magnitude = self.find_distance_to_point(self.current_position, self.goal_position)
            u = (v[0] / v_magnitude, v[1] / v_magnitude)

            next_node = (int(self.current_position[0] + (100 * u[0])),
                         int(self.current_position[1] + (100 * u[1])))
            while next_node in self.local_environment or self.move_too_close_too_object(next_node,
                                                                                        self.local_environment):
                next_node = (next_node[0] + 10, next_node[1] + 10)
            self.intermediate_node = next_node
        else:
            for i in range(len(possible_nodes)):
                if next_node == None:
                    next_node = possible_nodes[i]
                else:
                    if possible_nodes[i][1] > next_node[1]:
                        next_node = possible_nodes[i]
            self.intermediate_node = next_node[0]







    def move_too_close_too_object(self, point, recent_data):
        """

        :param point:
        :param recent_data:
        :return:
        """

        for data in recent_data:
            if self.find_distance_to_point(point, data) < 10:
                return True
        return False

    def generate_possible_moves(self, position):
        """
        This function generates the 8 possible moves around the current position and returns them as a list
        :return: A list of 8 next positions
        """
        possible_moves = self.calculate_points_around_a_point(position)
        return possible_moves

    @staticmethod
    def calculate_points_around_a_point(p):
        """

        :param p:
        :return:
        """
        points = [((p[0] - 1), (p[1] + 1)), ((p[0]), (p[1] + 1)), ((p[0] + 1), (p[1] + 1)),
                  ((p[0] - 1), p[1]), ((p[0] + 1), p[1]),
                  ((p[0] - 1), (p[1] - 1)), ((p[0]), (p[1] - 1)), ((p[0] + 1), (p[1] - 1))]
        return points

    def find_distance_to_point(self, next_position, goal_position):
        """
        Finds the euclidian distance between a next position and the goal node

        :param goal_position:
        :param next_position: One of the next positions from the list
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
                drone.add_data_to_local_env(self.local_environment)

    def check_env_for_drones(self):
        """

        :return:
        """
        local_drones = []
        for drone in self.environment_drones:
            if drone.id != self.id:
                if self.find_distance_to_point(self.current_position, drone.current_position) < 50:
                    local_drones.append(drone)
                    # print("drone " + str(self.id) + " is connecting with drone " + str(drone.id))
        return local_drones

    def communicate_to_ground_station(self):
        """
        Communications to the ground station, all drones can currently do this
        :return: None
        """
        self.ground_station.combine_data(self.local_environment, self.current_position, self.previous_position, self.checked_nodes)

    def add_data_to_local_env(self, data):
        """

        :param data:
        :return:
        """
        if self.local_environment:
            self_local_np = np.array(self.local_environment)
            other_local_np = np.array(data)
            concat = np.concatenate((self_local_np, other_local_np), axis=0)
            remove_duplicates = np.unique(concat, axis=0)
            self.local_environment = remove_duplicates.tolist()
        else:
            self.local_environment = data
