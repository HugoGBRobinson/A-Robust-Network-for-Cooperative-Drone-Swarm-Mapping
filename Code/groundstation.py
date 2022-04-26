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
        self.mapping_chunks = []
        self.mapped_chunks = []
        self.chunk_environment()


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

    def vertical_linear_exploration(self, drones):
        if drones is None:
            drones = self.environment.drones
        a = int(len(self.chunks) / self.number_of_drones)
        for i in range(self.number_of_drones):
            for ii in range(a):
                chunk = self.chunks.pop(0)
                self.send_chunks_to_drone(chunk, drones[i])
                self.mapping_chunks.append(chunk)

    def horizontal_linear_exploration(self, drones):
        if drones is None:
            drones = self.environment.drones
        self.chunks = []
        x_max = self.environment.mapw
        y_max = self.environment.maph

        for i in range(0, y_max, 100):
            column = []
            for ii in range(0, x_max, 100):
                column.append((ii, ii + 100, i, i + 100))
            self.chunks.append(column)

        a = int(len(self.chunks) / self.number_of_drones)
        for i in range(self.number_of_drones):
            for ii in range(a):
                chunk = self.chunks.pop(0)
                self.send_chunks_to_drone(chunk, drones[i])
                self.mapping_chunks.append(chunk)


    def out_in_exploration(self, drones):
        if drones is None:
            drones = self.environment.drones
        clockwise = True
        for drone in drones:
            if len(drones) > 1:
                if clockwise:
                    drone.set_chunks([self.chunks[0][0], self.chunks[0][int(self.environment.maph / 100) - 1],
                                          self.chunks[int(self.environment.mapw / 100) - 1][
                                              int(self.environment.maph / 100) - 1]])
                    clockwise = False
                else:
                    drone.set_chunks([self.chunks[0][0], self.chunks[int(self.environment.mapw / 100) - 1][0],
                                          self.chunks[int(self.environment.mapw / 100) - 1][
                                              int(self.environment.maph / 100) - 1]])
                    clockwise = True
            else:
                drone = drones[0]
                drone.set_chunks([self.chunks[0][0], self.chunks[int(self.environment.mapw / 100) - 1][0],
                                  self.chunks[int(self.environment.mapw / 100) - 1]
                                  [int(self.environment.maph / 100) - 1],
                                  self.chunks[0][int(self.environment.maph / 100) - 1], self.chunks[0][0]])

        self.chunks.pop()
        self.chunks.pop(0)
        for chunk in self.chunks:
            chunk.pop()
            chunk.pop(0)
        for drone in drones:
            for ii in range(10):
                self.send_chunks_to_drone([self.chunks[random.randint(0, len(self.chunks) - 1)]
                                           [random.randint(0, len(self.chunks[0]) - 1)]], drone)

        return False

    def random_exploration(self, drones):
        for i in range(len(drones)):
            for ii in range(30):
                self.send_chunks_to_drone([self.chunks[random.randint(0, len(self.chunks) - 1)]
                                           [random.randint(0, len(self.chunks[0]) - 1)]], drones[i])

    def mixed_exploration(self, drones):
        if drones is None:
            drones = self.environment.drones
        non_random_drones = []
        if self.number_of_drones >= 4:
            for i in range(len(drones)):
                if i % 4 == 0:
                    non_random_drones.append(drones[i])
                else:
                    self.random_exploration([drones[i]])
        else:
            self.random_exploration(drones)
        if len(non_random_drones) > 0:
            self.out_in_exploration(non_random_drones)

    def send_chunks_to_drone(self, chunks, drone):
        drone.set_chunks(chunks)



