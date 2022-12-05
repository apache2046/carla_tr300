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

https://user-images.githubusercontent.com/1034542/205655694-168f8995-d245-402d-8d12-c2ccbe8c9787.webm

https://user-images.githubusercontent.com/1034542/205655718-3063b1ab-021c-458f-84db-e47bcdd0820c.webm
