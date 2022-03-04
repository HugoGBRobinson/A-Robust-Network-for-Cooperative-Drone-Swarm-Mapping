import math

import numpy as np
import pygame


class Sensor:
    def __init__(self, range, map):
        """
        Constructor for the sensor class
        :param range: The range of the sensor
        :param map: The raw map data
        """
        self.Range = range
        self.speed = 4  # Rotations per second
        self.position = (0, 0)
        self.map = map
        self.W, self.H = pygame.display.get_surface().get_size()
        self.sensedObstacles = []

    def distance(self, obstacle_position):
        """
        Euclidian distance
        :param obstacle_position: The 2D coordinates for the obstacle
        :return: The Euclidian distance
        """
        px = (obstacle_position[0] - self.position[0]) ** 2
        py = (obstacle_position[1] - self.position[1]) ** 2
        return math.sqrt(px + py)

    def sense_obstacles(self, current_position):
        """
        A function to simulate the lidar finder on the drone to navigate the environment
        :return: Returns the point cloud data
        """
        self.position = current_position
        data = []
        x1, y1 = self.position[0], self.position[1]
        for angle in np.linspace(0, 2 * math.pi, 200, False):
            x2, y2 = (x1 + self.Range * math.cos(angle), y1 - self.Range * math.sin(angle))
            for i in range(0, 100):
                # Interpolation
                u = i / 100
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                # If within the window
                if 0 < x < self.W and 0 < y < self.H:
                    colour = self.map.get_at((x, y))
                    if (colour[0], colour[1], colour[2]) == (0, 0, 0):
                        distance = self.distance((x, y))
                        output = [distance, angle, self.position]
                        # Store the measurement
                        data.append(output)
                        break
        if len(data) > 0:
            return data
        else:
            return False
