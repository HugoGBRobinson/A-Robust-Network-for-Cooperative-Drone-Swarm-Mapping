import pygame

import drone
import env
import groundstation
import lidar


def main():
    count = 0
    environment = env.BuildEnvironment((600, 1200))
    environment.originalMap = environment.map.copy()
    environment.map.fill((0, 0, 0))
    environment.infomap = environment.map.copy()

    num_of_drones = 5

    ground_station = groundstation.GroundStation(environment, num_of_drones)


    drones = []
    drone_deflects_clockwise = False
    for i in range(num_of_drones):
        drones.append(
            drone.Drone(i, (100, 100), lidar.Sensor(200, pygame.surfarray.array2d(environment.originalMap)),
                        environment.drones, ground_station, environment, drone_deflects_clockwise)) #environemnt added for testing
        if drone_deflects_clockwise == True:
            drone_deflects_clockwise = False
        else:
            drone_deflects_clockwise = True

    running = True

    environment.set_drones_in_env(drones)
    ground_station.mixed_exploration(drones=None)
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
            percentage = pecentage_map_explored(environment.originalMap, environment.infomap)
            print(percentage)
            if percentage > 90:
                print("-----------------------------------------------------------------------------------------------")
                print("The " + str(num_of_drones) + " drone(s) explored 90% of the environment in " + str(count)
                      + " iterations")
                print("-----------------------------------------------------------------------------------------------")
                break
        count += 1


def pecentage_map_explored(whole_map, current_map):
    whole_map = pygame.surfarray.pixels2d(whole_map)
    # 16711680
    whole_map_count = list(whole_map.flatten()).count(0)
    current_map = pygame.surfarray.pixels2d(current_map)
    current_map = list(current_map.flatten())
    current_map_count = len([colour for colour in current_map if colour == 16711680])
    return ((current_map_count / whole_map_count) * 100)


def run_drones(drone):
    drone.sense_environment()


if __name__ == '__main__':
    main()
