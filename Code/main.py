import env, lidar
import pygame


environment = env.buildEnvironment((600,1200))
environment.origionalMap = environment.map.copy()
laser = lidar.sensor(200, environment.origionalMap)
environment.map.fill((0,0,0))
environment.infomap = environment.map.copy()

running = True

while running:
    sensorOn = False
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if pygame.mouse.get_focused():
            sensorOn = True
        elif not pygame.mouse.get_focused():
            sensorOn = False

    if sensorOn:
        position = pygame.mouse.get_pos()
        laser.position = position
        sensor_data = laser.sense_obstacles()
        environment.dataStorage(sensor_data)
        environment.show_lidarData()
    environment.map.blit(environment.infomap, (0,0))
    pygame.display.update()