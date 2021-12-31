import env, lidar, drone
import pygame

environment = env.buildEnvironment((600, 1200))
environment.origionalMap = environment.map.copy()
environment.map.fill((0, 0, 0))
environment.infomap = environment.map.copy()

num_of_drones = 1
drones = []
for i in range(num_of_drones):
    drones.append(drone.drone(i, [100, 100], lidar.sensor(200, environment.origionalMap)))
running = True

while running:
    sensorOn = True

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    if sensorOn:
        drone_positions = []
        for i in range(len(drones)):
            drones[i].sense_environment()
            environment.pointCloud = drones[i].local_environment
            drone_positions.append(drones[i].position)
            environment.show_lidarData(drone_positions)
        environment.map.blit(environment.infomap, (0, 0))
        pygame.display.update()


