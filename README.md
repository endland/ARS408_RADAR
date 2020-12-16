# ARS408_RADAR
This is based on AutowareClass2020 Radar detection. (https://gitlab.com/ApexAI/autowareclass2020/-/tree/master/lectures/09_Perception_Radar)

I tested it on ROS2 foxy in Ubutu 20.04. (Not ADE)


```
$ colcon build
```
Virtual CAN Setup


```
$ sudo modprobe vcan
$ sudo ip link add dev can0 type vcan
$ sudo ip link set up can0
$ candump can0
```
Play CAN dumpfile in CAN_Recording folder
```
$ cd CAN_recording
$ canplayer -I candump.log -l i
```

Launch Radar node, configure and activate
```
$./build/ars408_radar/ars408_radar_composition
$ ros2 lifecycle set /ars408_radar configure
$ ros2 lifecycle set /ars408_radar activate
```
Launch Rviz
```
$ rviz2
```

In RVIZ2 you have to add the marker array and set the global options to the radar_link:

add -> marker array (/radar/marker_array)

add -> Axes (radar_link)

Global Options -> Fixed Frame = radar_link
