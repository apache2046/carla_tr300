# carla_tr300
use thrustmaster tr300 to control vehicles in carla simulator


## Record trajectory with human drive
1. start carla first, then
2. ``` python manual_control.py --filter model3 --res 1920x1080 ```

## track the recorded trajectory
1. start carla first , then

2. ``` python track_img.py ```

## draw overlapped trace
1. save a log, modifiy it
2. ``` python draw_track.py ```
