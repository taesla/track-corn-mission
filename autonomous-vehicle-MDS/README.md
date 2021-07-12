# autonomous-vehicle-MDS
``2020-07-09``


## Commands
### reviced Commands for versionE - 1

``roslaunch stauto_sensor stauto_setting.launch``

``roslaunch stauto_sensor versionE.launch``

``rosrun stauto_sensor get_gps_data.py``

``roslaunch stauto_core stauto_core.launch``

``roslaunch mapviz mapviz.launch``

``roslaunch velodyne_pointcloud VLP16_points.launch``

``roslaunch adaptive_clustering adaptive_clustering.launch``

``rosrun stauto_sensor camera.py``

``roslaunch darknet_ros deeplearning.launch``

``rosrun stauto_sensor parking_move.py``

``rosrun adaptive_clustering parkin.py``


### Commands for versionE

``roslaunch stauto_sensor stauto_setting.launch``

``roslaunch stauto_sensor versionE.launch``

``rosrun stauto_sensor get_gps_data.py``

``roslaunch stauto_core stauto_core.launch``

``roslaunch mapviz mapviz.launch``

``roslaunch velodyne_pointcloud VLP16_points.launch``

``roslaunch adaptive_clustering adaptive_clustering.launch``

``rosrun teb_local_planner_tutorials cmd_vel_to_ackermann_drive.py``


### Commands Pure_Pursuit Control VersionC

``roslaunch stauto_senor stauto_setting.launch``

``rosrun stauto_senor get_gps_data.py``

``rosrun stauto_senor get_imu_24gv4.py`` or ``rosrun stauto_senor get_imu_24gv2.py``

``rosrun stauto_senor gps_path_pure_pursuit3.py``

``rosrun stauto_senor gps_control2.py``

### Velodyne lidar & IMU & ERP_to_PC & GPS 

``roslaunch stauto_senor stauto_setting.launch``

### USB cam

``roslaunch usb_cam usb_cam-test.launch ``


### Velodyne lidar

``roslaunch velodyne_pointcloud VLP16_points.launch``


### IMU

``rosrun  stauto_sensor get_imu_24gv4.py 
``

### Getting ERP42 information (encoder, steer, speed, brake, gear)

``rosrun  stauto_control ERP42_to_PC
``

### GPS

``rosrun  stauto_sensor get_gps_data.py 
``

### GPS Save

``rosrun stauto_sensor gps_data_save.py 
``

### GPS Save from rosbag to txt

``rosrun stauto_sensor save_gps_from_rosbag.py
``

### GPS Path

``rosrun stauto_sensor save_gps_from_rosbag.py
``

### Lidar + camera fusion

`` roslaunch darknet_ros yolo_v3.launch ``
``roslaunch cam_lidar_calib cam_lidar_proj_basler.launch``

### localization

``roslaunch stauto_sensor STauto_Racing.launch``

param

``pub_wheel_odom_tf:=true``: wheel_odom tf publish 결정

``use_amcl``:amcl 사용결정

## ROSBAG save

``rosbag record -a ``


## ROSBAG play

``rosbag play -($rosbag name).bag --clock``

## Protocol

![img](./docs/Protocol_set.png)
