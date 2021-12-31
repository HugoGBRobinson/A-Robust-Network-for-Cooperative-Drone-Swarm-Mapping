import env, lidar, drone
import pygame

environment = env.buildEnvironment((600, 1200))
environment.origionalMap = environment.map.copy()
# Legacy code for using the mouse as the sensor
# laser = lidar.sensor(200, environment.origionalMap)
drone = drone.drone(1, [100, 100], lidar.sensor(200, environment.origionalMap))
environment.map.fill((0, 0, 0))
environment.infomap = environment.map.copy()

running = True
count = 0

while count < 1000:
    sensorOn = True

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    # Legacy code for using the mouse as the sensor
    #     if pygame.mouse.get_focused():
    #         sensorOn = True
    #     elif not pygame.mouse.get_focused():
    #         sensorOn = False

    if sensorOn:
        # Legacy code for using the mouse as the sensor
        # position = pygame.mouse.get_pos()
        # laser.position = position
        # l = laser.sense_obstacles()
        environment.dataStorage(drone.sense_environment())
        environment.show_lidarData()
    environment.map.blit(environment.infomap, (0, 0))
    pygame.display.update()
    count += 1
