import pygame


class BuildEnvironment:
    def __init__(self, map_dimensions):
        """
        The constructor for the environment
        :param map_dimensions: A list containing 2 elements of the map width and height
        """
        pygame.init()
        self.pointCloud = []
        self.externalMap = pygame.image.load('Floor Plan.png')
        self.maph, self.mapw = map_dimensions
        self.MapWindowName = 'Point Cloud Test'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.mapw, self.maph))
        self.map.blit(self.externalMap, (0, 0))
        self.infomap = self.map.copy()

    def show_lidar_data(self, global_env, positions):
        """
        A function to output the lidar data and positions of drones to the infomap
        :param global_env: The global environment from the ground_station
        :param positions: A 2D positions of all the drones
        :return: None
        """

        for point in global_env:
            self.infomap.set_at(point, (255, 0, 0))
        for position in positions:
            self.infomap.set_at(position, (0, 255, 0))
