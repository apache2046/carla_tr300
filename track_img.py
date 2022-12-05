# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob 
import os
import sys
import cv2
import math
import csv
import carla
from carla import ColorConverter as cc
import time
import numpy as np
import controller2d_stanley

INTERP_LOOKAHEAD_DISTANCE = 20   # lookahead in meters
INTERP_DISTANCE_RES       = 0.01 # distance between interpolated points


client = carla.Client('192.168.31.6', 2000)
client.set_timeout(20.0)
# client.load_world('Town07')
world = client.get_world()

original_settings = world.get_settings()
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 1.0 / 60 #0.05
world.apply_settings(settings)
# client.reload_world(False) # reload map keeping the world settings

bp_library = world.get_blueprint_library()
map = world.get_map()
spawn_point_transform = carla.Transform()
spawn_point_transform.location.x = -3 #-198.5 #-176
spawn_point_transform.location.y = -87 #-145 #111
spawn_point_transform.location.z = 0.1
spawn_point_transform.rotation.yaw = 90
spectator = world.get_spectator()
vehicle_bp = bp_library.filter('model3')[0]
player = world.spawn_actor(vehicle_bp, spawn_point_transform)


camera_bp = bp_library.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '1920')
camera_bp.set_attribute('image_size_y', '1080')
camera_bp.set_attribute('fov', "110")
# camera_bp.set_attribute('sensor_tick', str(1/60))

camera_sensor = world.spawn_actor(camera_bp,
                carla.Transform(carla.Location(x=-6, y=0, z=3), carla.Rotation(pitch=15, yaw=0)),
                attach_to=player,
                attachment_type= carla.AttachmentType.SpringArm)
camera_surface = None
img_idx = 0
def parse_img(image):
    global camera_surface
    print('parse_img', camera_surface, image.height, image.width)
    image.convert(cc.Raw)
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    cvimg = array[:, :, :3]
    #array = array[:, :, ::-1]
    global img_idx
    # font
    font = cv2.FONT_HERSHEY_SIMPLEX
    org = (50, 50)
    fontScale = 1
    color = (255, 255, 255)
    thickness = 2
    global display_speed
    cvimg = cv2.putText(cvimg, f'{int(display_speed * 3.6)}km/h', org, font, 
                    fontScale, color, thickness, cv2.LINE_AA)
    cv2.imwrite(f'/tmp/imgs/{img_idx:04}.png', cvimg)
    print(img_idx)
    img_idx += 1
    

camera_sensor.listen(lambda image: parse_img(image))

world_tick = 0
world.tick()

with open("waypoint3.txt") as waypoints_file_handle:
    waypoints = list(csv.reader(waypoints_file_handle, 
                                delimiter=',',
                                quoting=csv.QUOTE_NONNUMERIC))
    waypoints_np = np.array(waypoints)[70:3260]
print(waypoints_np[:3])
controller = controller2d_stanley.Controller2D(waypoints_np)
throttle, steer, brake = 0, 0, 0
while True:
    if world_tick == img_idx:
        time.sleep(0.002)
        continue
    else:
        world_tick += 1
    t = player.get_transform()
    v = player.get_velocity()
    c = player.get_control()
    speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)
    x = t.location.x
    y = t.location.y
    yaw = t.rotation.yaw

    if world_tick > 60:
        controller.update_values(x, y, math.radians(yaw), speed)
        controller.update_controls()
        throttle, steer, brake = controller.get_commands()
    print(x, y, speed, yaw, throttle, steer, brake)
    player.apply_control(carla.VehicleControl(throttle, steer, brake))
    
    world.tick()
