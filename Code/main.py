import pygame

import drone
import env
import groundstation
import lidar
import random


def main():
    """
    The main function sets up the environment and the drones.
    It also runs the simulation, keeping count of the number of iterations.
    :return: None
    """
    count = 0
    environment = env.BuildEnvironment((600, 1200))
    environment.originalMap = environment.map.copy()
    environment.map.fill((0, 0, 0))
    environment.infomap = environment.map.copy()

    num_of_drones = 10

    ground_station = groundstation.GroundStation(environment, num_of_drones)

    drones = []
    drone_deflects_clockwise = True
    for i in range(num_of_drones):
        drones.append(
            drone.Drone(i, (100, 100), lidar.Sensor(200, pygame.surfarray.array2d(environment.originalMap)),
                        environment.drones, ground_station, environment,
                        drone_deflects_clockwise))  # environment added for testing
        if drone_deflects_clockwise:
            drone_deflects_clockwise = False
        else:
            drone_deflects_clockwise = True

    running = True

    # Adds all the drones to the environment
    environment.set_drones_in_env(drones)
    # Sets the style of exploration
    ground_station.out_in_exploration(drones=drones)
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        for i in range(len(drones)):
            drones[i].sense_environment()
            ground_station.check_for_drones()
            environment.map.blit(environment.infomap, (0, 0))
            pygame.display.update()

        if count % 100 == 0:
            percentage = percentage_map_explored(environment.originalMap, environment.infomap)
            print(percentage)
            if percentage > 85:
                print("-----------------------------------------------------------------------------------------------")
                print("The " + str(num_of_drones) + " drone(s) explored 85% of the environment in " + str(count)
                      + " iterations")
                print("-----------------------------------------------------------------------------------------------")
                break
        if count == 10000:
            percentage = percentage_map_explored(environment.originalMap, environment.infomap)
            print("-----------------------------------------------------------------------------------------------")
            print("The " + str(num_of_drones) + " drone(s) explored + " + str(
                int(percentage)) + "% of the environment in " + str(count)
                  + " iterations")
            print("-----------------------------------------------------------------------------------------------")

        remove_drone(drones)
        if len(drones) == 0:
            percentage = percentage_map_explored(environment.originalMap, environment.infomap)
            print("-----------------------------------------------------------------------------------------------")
            print("The " + str(num_of_drones) + " drone(s) explored " + str(
                int(percentage)) + "% of the environment in " + str(count)
                  + " iterations before the swarm failed")
            print("-----------------------------------------------------------------------------------------------")
            break
        count += 1


def percentage_map_explored(whole_map, current_map):
    """
    This function works out the percentage of the map explored by the drones for the ground stations global map
    :param whole_map: The pygame surface of the underlying map
    :param current_map: The global environment of the ground station
    :return: A float of the percentage explored
    """
    whole_map = pygame.surfarray.pixels2d(whole_map)
    whole_map_count = list(whole_map.flatten()).count(0)
    current_map = pygame.surfarray.pixels2d(current_map)
    current_map = list(current_map.flatten())
    # 16711680 is the colour id for red
    current_map_count = len([colour for colour in current_map if colour == 16711680])
    return (current_map_count / whole_map_count) * 100


def remove_drone(drones):
    """
    A function to remove drones from the environment during the simulation, based on a percentage chance per iteration,
    this percentage chance is very low but can be changed
    :param drones: A list of the drones
    :return: None
    """
    attrition_rate = 500
    num = random.randint(0, attrition_rate)
    if num == 1:
        print("Removing drone")
        del(drones[0])


if __name__ == '__main__':
    main()
