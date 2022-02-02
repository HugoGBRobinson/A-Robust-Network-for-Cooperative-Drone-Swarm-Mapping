class GroundStation:
    def __init__(self, environment):
        """
        The constructor for the ground_station
        :param environment: The current environment
        """
        self.environment = environment
        self.global_environment = []
        self.drone_positions = []

    def combine_data(self, data, position, previous_position):
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

        self.environment.show_lidar_data(data, position, previous_position)
