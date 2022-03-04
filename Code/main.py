import pygame

import drone
import env
import groundstation
import lidar

import multiprocessing as mp

count = 0


def runDrones(drones):
    for i in range(len(drones)):
        drones[i].sense_environment()


if __name__ == '__main__':
    environment = env.BuildEnvironment((600, 1200))
    environment.originalMap = environment.map.copy()
    environment.map.fill((0, 0, 0))
    environment.infomap = environment.map.copy()

    ground_station = groundstation.GroundStation(environment)

    # pool = mp.Pool(mp.cpu_count())
    # pool = mp.Pool(1)

    num_of_drones = 3
    drones = []
    for i in range(num_of_drones):
        drones.append(
            drone.Drone(i, (100, 100), lidar.Sensor(200, environment.originalMap), ground_station, environment))
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
        for i in range(len(drones)):
            if sensorOn:
                #data = pool.apply_async(runDrones, drones).get()
                data = drones[i].sense_environment()
                ground_station.combine_data(data[0], data[1], data[2])
                environment.map.blit(environment.infomap, (0, 0))
                pygame.display.update()

        count += 1
