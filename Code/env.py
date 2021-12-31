import math
import pygame


class buildEnvironment:
    def __init__(self, MapDimensions):
        pygame.init()
        self.pointCloud = []
        self.externalMap = pygame.image.load('Floor Plan.png')
        self.maph, self.mapw = MapDimensions
        self.MapWindowName = 'Point Cloud Test'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.mapw, self.maph))
        self.map.blit(self.externalMap, (0, 0))
        # Colours
        self.black = (0, 0, 0)
        self.red = (255, 0, 0)
        self.white = (255, 255, 255)

    def show_lidarData(self, drone_positions):
        """
        A function to output the lidar data and positions of drones to the infomap
        :param drone_positions: A list of 2D drone positions
        :return: None
        """
        self.infomap = self.map.copy()
        for point in self.pointCloud:
            self.infomap.set_at((int(point[0]), int(point[1])), (255, 0, 0))
        for position in drone_positions:
            self.infomap.set_at(position, (0, 255, 0))
