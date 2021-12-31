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
        self.position = position
        self.sensor = sensor
        self.sensor_data = []
        self.ground_station = ground_station

    def sense_environment(self):
        """
        This function initiates one rotation from the simulated LIDAR sensor onboard the drone and returns the sensed
        obstacles
        :return: A list of sensed positions for the point cloud map
        """
        self.move()
        self.sensor.position = self.position
        self.data_storage(self.sensor.sense_obstacles())

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
        This function moves the drone, currently implements this movement as random locations in the environment

        Requires implementation of movement algorithm based of current local map
        :return: None
        """
        self.position = [random.randint(0, 1200), random.randint(0, 600)]

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
        self.ground_station.combine_data(self.local_environment, self.position)
