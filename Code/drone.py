import random
import math


class drone:
    """
    This class will encode a singular drone,its attributes and its functionality
    """

    def __init__(self, number, position, sensor):
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

    def sense_environment(self):
        """
        This function initiates one rotation from the simulated LIDAR sensor onboard the drone and returns the sensed
        obsticals
        :return: A list of sensed positions for the point cloud map
        """
        self.move()
        self.sensor.position = self.position
        self.dataStorage(self.sensor.sense_obstacles())

    def AD2pos(self, distance, angle, dronePosition):
        """
        A function to convert raw data to point on the map

        :param distance: The distance of the point from the drone
        :param angle: The angel from the drone
        :param dronePosition: The position of the drone
        :return: the integer location of the point in the environment
        """
        x = distance * math.cos(angle) + dronePosition[0]
        y = -distance * math.sin(angle) + dronePosition[1]
        return int(x), int(y)

    def dataStorage(self, data):
        """
        This function takes the raw sensor data, converts it to points on the map and saves i to the local environment

        :param data: The raw data from the sensor
        :return: None
        """
        if data is not False:
            for element in data:
                point = self.AD2pos(element[0], element[1], element[2])
                if point not in self.local_environment:
                    self.local_environment.append(point)
        else:
            print("False")

    def move(self):
        """
        This function moves the drone, currently implements this movement as random locations in the environment

        :return: None
        """
        self.position = [random.randint(0, 1200), random.randint(0, 600)]

    def communicate(self):
        """
        This function implements the communication protocols between the drones, currently not implemented

        :return: None
        """
        pass
