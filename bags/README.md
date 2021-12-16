# Bags Directory
This is the default directory for saving rosbag files recorded with the camera scripts in this package.

Bring the Logitech C920 webcam online to get a live video feed with the following commands:
```
roslaunch continuum_mocap c920_cam_online.launch
rqt
```

Record bags in your current directory with the following command:
```roslaunch continuum_mocap cam_record.launch duration:=<duration> file_name:="<file name>" path:=$PWD```
