import pyglet
from pyglet import shapes
import numpy as np
from pyglet.gl import *
import sys
from collections import deque
import csv
with open('waypoint3.txt') as fd:
    waypoints = list(csv.reader(fd, 
                                delimiter=',',
                                quoting=csv.QUOTE_NONNUMERIC))
    waypoints = np.array(waypoints)[:3330]
# print("G1", waypoints[:5])

win_width, win_height = 900 * 2, 800 * 2
scale_factor = 3.0 * 2

x_margin = - waypoints[:, 0].min() + 10
y_margin = - waypoints[:, 1].min() + 10

waypoints[:, 0] += x_margin
waypoints[:, 1] += y_margin
waypoints[:, 1] = win_height / scale_factor - waypoints[:, 1]

window = pyglet.window.Window(win_width, win_height, resizable=True)

batch1 = pyglet.graphics.Batch()
batch2 = pyglet.graphics.Batch()

ori_trace = []
for i in range(len(waypoints)-1):
    p1 = waypoints[i]
    p2 = waypoints[i+1]
    ori_trace.append(shapes.Line(p1[0]*scale_factor, p1[1]*scale_factor,
                             p2[0]*scale_factor, p2[1]*scale_factor, 
                             width=2, color=(0, 255, 0), batch=batch1))

with open('log2') as fd:
    new_waypoints = list(csv.reader(fd, 
                                delimiter=' ',
                                quoting=csv.QUOTE_NONNUMERIC))
    new_waypoints = np.array(new_waypoints)
print("G2", new_waypoints[:5])
new_waypoints[:, 0] += x_margin
new_waypoints[:, 1] += y_margin
new_waypoints[:, 1] = win_height / scale_factor - new_waypoints[:, 1]

for item in new_waypoints:
    if len(item) != 7:
        print('bad', item)
        sys.exit()
new_trace = []

frame = 0
# batch1 = pyglet.graphics.Batch()
@window.event
def on_draw():
    global frame
    # # glScalef(2.0, 2.0, 2.0)
    glClearColor(1, 1, 1, 1)

    frame += 1
    if frame == len(new_waypoints)-2:
        sys.exit()
    # print(frame)
    p1 = new_waypoints[frame]
    p2 = new_waypoints[frame+1]
    new_trace.append(shapes.Line(p1[0]*scale_factor, p1[1]*scale_factor,
                            p2[0]*scale_factor, p2[1]*scale_factor, 
                            width=2, color=(255, 0, 0), batch=batch2))

    window.clear()
    batch1.draw()
    batch2.draw()

    pyglet.image.get_buffer_manager().get_color_buffer().save(f'/tmp/imgs2/{frame:04}.png')

pyglet.app.run()