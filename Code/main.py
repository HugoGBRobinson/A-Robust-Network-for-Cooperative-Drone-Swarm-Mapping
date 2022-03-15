import multiprocessing

import pygame

import drone
import env
import groundstation
import lidar

import multiprocessing as mp

count = 0

if __name__ == '__main__':
    environment = env.BuildEnvironment((600, 1200))
    environment.originalMap = environment.map.copy()
    environment.map.fill((0, 0, 0))
    environment.infomap = environment.map.copy()

    ground_station = groundstation.GroundStation(environment)

    num_of_drones = 1
    drones = []
    for i in range(num_of_drones):
        drones.append(
            drone.Drone(i, (100, 100), lidar.Sensor(200, pygame.surfarray.array2d(environment.originalMap)),
                        environment.drones, ground_station))
    running = True

    environment.set_drones_in_env(drones)
    while running:
        if count < 10000:
            sensorOn = True
        else:
            sensorOn = False

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        if sensorOn:
            for i in range(len(drones)):
                drones[i].sense_environment()
                environment.map.blit(environment.infomap, (0, 0))
                pygame.display.update()

        count += 1
