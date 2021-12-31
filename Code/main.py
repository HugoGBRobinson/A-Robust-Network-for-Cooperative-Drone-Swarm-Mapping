import env, lidar, drone, ground_station
import pygame

environment = env.buildEnvironment((600, 1200))
environment.originalMap = environment.map.copy()
environment.map.fill((0, 0, 0))
environment.infomap = environment.map.copy()

ground_station = ground_station.ground_station(environment)

num_of_drones = 1
drones = []
for i in range(num_of_drones):
    drones.append(drone.drone(i, [100, 100], lidar.sensor(200, environment.originalMap), ground_station))
running = True

count = 0

while running:
    if count < 1000:
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


