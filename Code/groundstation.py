import random

class GroundStation:
    def __init__(self, environment, number_of_drones):
        """
        The constructor for the ground_station
        :param environment: The current environment
        """
        self.environment = environment
        self.global_environment = []
        self.number_of_drones = number_of_drones
        self.drone_positions = []
        self.chunks = []
        self.chunk_environment()
        self.mapping_chunks = []
        self.mapped_chunks = []

    def combine_data(self, data, position, previous_position, checked_nodes, intermediate_node):
        """
        Each drone calls this function to combine their local data to the global data, subject to change
        :param data: Local data from drone
        :param position: Current position of drone
        :return: None
        """
        self.global_environment.extend(data)
        if len(self.drone_positions) > 0:
            self.drone_positions.pop()
        self.drone_positions.append(position)

        self.environment.show_lidar_data(data, position, previous_position, checked_nodes, intermediate_node)

    def chunk_environment(self):
        x_max = self.environment.mapw
        y_max = self.environment.maph

        for i in range(0, x_max, 100):
            column = []
            for ii in range(0, y_max, 100):
                column.append((i, i + 100, ii, ii + 100))
            self.chunks.append(column)

    def vertical_linear_exploration(self):
        a = int(len(self.chunks) / self.number_of_drones)
        for i in range(self.number_of_drones):
            for ii in range(a):
                chunk = self.chunks.pop(0)
                self.send_chunks_to_drone(chunk, self.environment.drones[i])
                self.mapping_chunks.append(chunk)

    def horizontal_linear_exploration(self):
        horizontal_chunk = []
        for i in range(self.chunks):
            horizontal_chunk.append(self.chunks[i].pop)
        return False

    def out_in_exploration(self):
        return False

    def random_exploration(self):
        for i in range(self.number_of_drones):
            for ii in range(30):
                self.send_chunks_to_drone([self.chunks[random.randint(0, len(self.chunks) - 1)]
                                           [random.randint(0, len(self.chunks[0]) - 1)]], self.environment.drones[i])

    def mixed_exploration(self):
        return False

    def send_chunks_to_drone(self, chunks, drone):
        drone.set_chunks(chunks)
        # Communication through a dict
        # Key is drone number
        # Value is list of chunks to explore
