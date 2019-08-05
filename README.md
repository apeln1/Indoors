# Indoors


## Docker

### running docker with GUI
```
QT_GRAPHICSSYSTEM="native" docker run --name=ros_with_share -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/catkin_src:/root/catkin_ws/src 057
```
### container exec
```
docker exec -it 154 bash
```
