# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob 
import os
import sys
import cv2
import math


try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass 

import carla
from carla import ColorConverter as cc
import time
import numpy as np

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
spawn_point_transform.location.z = 3
spawn_point_transform.rotation.yaw = 90
spectator = world.get_spectator()
vehicle_bp = bp_library.filter('model3')[0]
player = world.spawn_actor(vehicle_bp, spawn_point_transform)


camera_bp = bp_library.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '1280')
camera_bp.set_attribute('image_size_y', '720')
camera_bp.set_attribute('fov', "100")
camera_bp.set_attribute('sensor_tick', str(1/60))

camera_sensor = world.spawn_actor(camera_bp,
                carla.Transform(carla.Location(x=-6, y=0, z=3), carla.Rotation(pitch=0, yaw=0)),
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
    array = array[:, :, :3]
    #array = array[:, :, ::-1]
    global img_idx
    cv2.imwrite(f'/tmp/imgs/{img_idx:04}.png', array)
    print(img_idx)
    img_idx += 1
    

camera_sensor.listen(lambda image: parse_img(image))

while True:
    time.sleep(0.2)
    world.tick()
    t = player.get_transform()
    v = player.get_velocity()
    c = player.get_control()
    speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)
    print(t.location.x, t.location.y, speed)
