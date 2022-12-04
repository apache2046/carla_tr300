import carla
from carla import ColorConverter as cc
import pygame
import time
import numpy as np

client = carla.Client('127.0.0.1', 2000)
client.set_timeout(20.0)
# client.load_world('Town07')
world = client.get_world()

original_settings = world.get_settings()
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 1.0 / 60 #0.05
world.apply_settings(settings)
client.reload_world(False) # reload map keeping the world settings

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
camera_sensor = world.spawn_actor(camera_bp,
                carla.Transform(carla.Location(x=-8, y=0, z=5), carla.Rotation(pitch=0, yaw=0)),
                attach_to=player,
                attachment_type= carla.AttachmentType.SpringArm)
camera_surface = None
def parse_img(image):
    global camera_surface
    print('parse_img', camera_surface, image.height, image.width)
    image.convert(cc.Raw)
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    camera_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

camera_sensor.listen(lambda image: parse_img(image))

pygame.init()
pygame.font.init()
display = pygame.display.set_mode((1280,720), pygame.HWSURFACE | pygame.DOUBLEBUF)
display.fill((0,0,0))
pygame.display.flip()
while True:
    time.sleep(0.01)
    world.tick()
    if camera_surface is not None:
        display.blit(camera_surface, (0, 0))
    pygame.display.flip()