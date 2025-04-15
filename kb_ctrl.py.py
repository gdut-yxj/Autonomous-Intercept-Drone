import sys
import time
import airsim
import pygame

pygame.init()
screen = pygame.display.set_mode((640, 480))
pygame.display.set_caption('keyboard ctrl')
screen.fill((0, 0, 0))

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()

base_vel = 2.0
acc = 10.0
flag = False
base_yaw_vel = 5.0

goal = airsim.Vector3r(46.0, 78.0, -0.5)

while True:
    yaw_rate = 0.0
    v_x = 0.0
    v_y = 0.0
    v_z = 0.0
    time.sleep(0.02)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()
    scan_wrapper = pygame.key.get_pressed()
    if scan_wrapper[pygame.K_SPACE]:
        scale_ratio = acc
    else:
        scale_ratio = 1.0
    if scan_wrapper[pygame.K_a] or scan_wrapper[pygame.K_d]:
        yaw_rate = (scan_wrapper[pygame.K_d] - scan_wrapper[pygame.K_a]) * scale_ratio * base_yaw_vel
    if scan_wrapper[pygame.K_UP] or scan_wrapper[pygame.K_DOWN]:
        v_x = (scan_wrapper[pygame.K_UP] - scan_wrapper[pygame.K_DOWN]) * scale_ratio
    if scan_wrapper[pygame.K_LEFT] or scan_wrapper[pygame.K_RIGHT]:
        v_y = -(scan_wrapper[pygame.K_LEFT] - scan_wrapper[pygame.K_RIGHT]) * scale_ratio
    if scan_wrapper[pygame.K_w] or scan_wrapper[pygame.K_s]:
        v_z = -(scan_wrapper[pygame.K_w] - scan_wrapper[pygame.K_s]) * scale_ratio
    client.moveByVelocityBodyFrameAsync(vx=v_x, vy=v_y, vz=v_z, duration=0.02, yaw_mode=airsim.YawMode(True, yaw_or_rate=yaw_rate))
    pygame.event.pump()
    quad_pos = client.getMultirotorState().kinematics_estimated.position
    distance = quad_pos.distance_to(goal)
    print(quad_pos.x_val, quad_pos.y_val, quad_pos.z_val)
    if scan_wrapper[pygame.K_ESCAPE]:
        pygame.quit()
        sys.exit()
