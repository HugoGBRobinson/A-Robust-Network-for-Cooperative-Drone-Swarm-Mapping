import math
import random


class Drone:
    """
    This class will encode a singular drone,its attributes and its functionality
    """

    def __init__(self, number, position, sensor, ground_station):
        """
        The constructor for the drone class
        :param number: The id number of the drone
        :param position: Where the drone is in the environment
        :param sensor: The lidar class
        """
        self.number = number
        self.local_environment = []
        self.current_position = position
        self.sensor = sensor
        self.sensor_data = []
        self.ground_station = ground_station
        self.goal_position = [random.randint(0, 1200), random.randint(0, 600)]

    def sense_environment(self):
        """
        This function initiates one rotation from the simulated LIDAR sensor onboard the drone and returns the sensed
        obstacles
        :return: A list of sensed positions for the point cloud map
        """
        self.move()
        self.sensor.current_position = self.current_position
        self.data_storage(self.sensor.sense_obstacles(self.current_position))

        self.communicate_to_ground_station()

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

    def move(self):
        """
        This function moves the drone, currently implements this movement as an A* algorithm, however does not avoid
        obsticals and does not reassign goal nodes
        :return: None
        """
        possible_moves = self.generate_possible_moves()
        shortest_distance = 10000000
        next_move = []
        for move in possible_moves:
            distance = self.find_distance_to_goal(move)
            if distance < shortest_distance:
                shortest_distance = distance
                next_move = move
        if self.current_position == self.goal_position:
            self.current_position = next_move
            self.goal_position = [random.randint(0, 1200), random.randint(0, 600)]
        else:
            self.current_position = next_move

    def generate_possible_moves(self):
        """
        This function generates the 8 possible moves around the current position and returns them as a list
        :return: A list of 8 next positions
        """
        p = self.current_position
        return [[(p[0] - 1), (p[1] + 1)], [(p[0]), (p[1] + 1)], [(p[0] + 1), (p[1] + 1)],
                [(p[0] - 1), p[1]], [(p[0] + 1), p[1]],
                [(p[0] - 1), (p[1] - 1)], [(p[0]), (p[1] - 1)], [(p[0] + 1), (p[1] - 1)]]

    def find_distance_to_goal(self, next_position):
        """
        Finds the euclidian distance between a next position and the goal node

        :param next_position: One of the next positions from the list
        :return: The euclidian distance
        """
        x1 = next_position[0]
        y1 = next_position[1]
        x2 = self.goal_position[0]
        y2 = self.goal_position[1]
        return math.sqrt(math.pow((x2 - x1), 2) + math.pow((y2 - y1), 2))

    def communicate_to_drone(self):
        """
        This function implements the communication protocols between the drones, currently not implemented

        Requires implementation of communications to other drones to update local map
        :return: None
        """
        pass

    def communicate_to_ground_station(self):
        """
        Communications to the ground station, all drones can currently do this
        :return: None
        """
        self.ground_station.combine_data(self.local_environment, self.current_position)
