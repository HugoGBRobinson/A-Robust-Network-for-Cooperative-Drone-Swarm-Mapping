import env


class ground_station:
    def __init__(self, environment):
        self.environment = environment
        self.global_environment = []
        self.drone_positions = []

    def combine_data(self, data, position):
        self.global_environment.extend(data)
        if len(self.drone_positions) > 0:
            self.drone_positions.pop()
        self.drone_positions.append(position)

        self.environment.show_lidarData(self.global_environment, self.drone_positions)
