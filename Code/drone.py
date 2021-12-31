import env, lidar
import pygame
import random


class drone:
    """
    This class will encode a singular drone,its attributes and its functionality
    """

    def __init__(self, number, position, sensor):
        """
        The constructor for the drone class
        :param number:
        :param position:
        :param sensor:
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
        return self.sensor.sense_obstacles()

    def move(self):
        """
        This function moves the drone, currently implements this movement as random locations in the environment
        :return: Nothing
        """
        self.position = [random.randint(0, 1200), random.randint(0, 1200)]

    def communicate(self):
        """
        This function implements the communication protocols between the drones, currently not implemented
        :return: Nothing
        """
        pass
