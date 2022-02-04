import pygame

import drone
import env
import groundstation
import lidar

environment = env.BuildEnvironment((600, 1200))
environment.originalMap = environment.map.copy()
environment.map.fill((0, 0, 0))
environment.infomap = environment.map.copy()

ground_station = groundstation.GroundStation(environment)

num_of_drones = 10
drones = []
for i in range(num_of_drones):
    drones.append(drone.Drone(i, (100, 100), lidar.Sensor(200, environment.originalMap), ground_station, environment))
running = True

environment.set_drones_in_env(drones)


count = 0

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
