import math
import random



class Drone:
    """
    This class will encode a singular drone,its attributes and its functionality
    """

    def __init__(self, id, position, sensor, ground_station, environment):
        """
        The constructor for the drone class
        :param number: The id number of the drone
        :param position: Where the drone is in the environment
        :param sensor: The lidar class
        """
        self.id = id
        self.local_environment = []
        self.previous_position = None
        self.current_position = position
        self.sensor = sensor
        self.sensor_data = []
        self.ground_station = ground_station
        self.goal_position = (random.randint(0, 1200), random.randint(0, 600))
        self.environment = environment


    def sense_environment(self):
        """
        This function initiates one rotation from the simulated LIDAR sensor onboard the drone and returns the sensed
        obstacles
        :return: A list of sensed positions for the point cloud map
        """

        self.sensor.current_position = self.current_position
        self.data_storage(self.sensor.sense_obstacles(self.current_position))

        self.communicate_to_ground_station()
        self.communicate_to_drone()
        self.move(self.local_environment[:50:-1])

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

        possible_moves = self.generate_possible_moves()
        shortest_distance = 10000000
        next_move = []
        buffered_possible_moves = []
        for move in possible_moves:
            if self.move_too_close_too_object(move, recent_data):
                buffered_possible_moves.append(move)

        for move in buffered_possible_moves:
            distance = self.find_distance_to_point(move, self.goal_position)
            if distance < shortest_distance:
                shortest_distance = distance
                next_move = move
        if self.previous_position == next_move:
            self.goal_position = (random.randint(0, 1200), random.randint(0, 600))
        if self.current_position == self.goal_position:
            self.previous_position = self.current_position
            self.current_position = next_move
            self.goal_position = (random.randint(0, 1200), random.randint(0, 600))
        else:
            self.previous_position = self.current_position
            self.current_position = next_move

    def move_too_close_too_object(self, point, recent_data):
        """

        :param point:
        :param recent_data:
        :return:
        """
        for data in recent_data:
            if self.find_distance_to_point(point, data) < 10:
                return False
        return True

    def generate_possible_moves(self):
        """
        This function generates the 8 possible moves around the current position and returns them as a list
        :return: A list of 8 next positions
        """
        p = self.current_position
        possible_moves = self.calculate_points_around_a_point(p)
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
        self.check_env_for_drones()

    def check_env_for_drones(self):
        for drone in self.environment.drones:
            if drone.id != self.id:
                if self.find_distance_to_point(self.current_position, drone.current_position) < 50:
                    print("drone " + str(self.id) + " is connecting with drone " + str(drone.id))

    def communicate_to_ground_station(self):
        """
        Communications to the ground station, all drones can currently do this
        :return: None
        """
        self.ground_station.combine_data(self.local_environment, self.current_position, self.previous_position)


