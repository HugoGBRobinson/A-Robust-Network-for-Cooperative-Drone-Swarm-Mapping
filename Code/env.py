import pygame


class buildEnvironment:
    def __init__(self, MapDimensions):
        """
        The constructor for the environment
        :param MapDimensions: A list containing 2 elements of the map width and height
        """
        pygame.init()
        self.pointCloud = []
        self.externalMap = pygame.image.load('Floor Plan.png')
        self.maph, self.mapw = MapDimensions
        self.MapWindowName = 'Point Cloud Test'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.mapw, self.maph))
        self.map.blit(self.externalMap, (0, 0))

    def show_lidarData(self, local_env, position):
        """
        A function to output the lidar data and positions of drones to the infomap
        :param position: A 2D position of the drone
        :param local_env: The local environment of the drone
        :return: None
        """
        self.infomap = self.map.copy()
        for point in local_env:
            self.infomap.set_at((int(point[0]), int(point[1])), (255, 0, 0))
        self.infomap.set_at(position, (0, 255, 0))
