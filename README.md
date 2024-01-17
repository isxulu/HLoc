# HLoc
For the MuYuan contract, improvements have been made to the Hloc algorithm.
1. Reconstruction: Block reconstruction and scene merging are aided by estimating camera poses from other sources.
2. Localization: A coarse-to-fine localization approach is adopted to enhance localization speed.

### Data collection platform

<img src=".\img\Data-collection-platform.jpg">

### Estimated Pose vs. Real Pose Comparison Chart

<img src=".\img\Estimated_Pose.png">

# Hardware operation commands

## Data Collection

### docker(env)

Creating a container through mirroring:

```bash
docker run -it  --net=host  \
    --name collect \ 
    -v /data:/data  \
    -v /tmp/.X11-unix:/tmp/.X11-unix  \
    -v /dev:/dev \
    --privileged=true \
    --ipc=host \
    -e DISPLAY=$DISPLAY \
   harbor.cvgl.lab/library/vslam-data-collection-platform:v1.1.3
```

Starting a container:

```bash
docker start collect
```

Running a Docker image:

```bash
docker exec -it collect bash  
```

### ROS

#### Basic Command

Configuring the workspace:

```bash
source /ws/install/setup.bash
```

Viewing all current topics:

```bash
ros2 topic list
```

Viewing the frequency of information sent for a specific topic:

```bash
ros2 topic hz /imu/data  
```

Printing the content of a specific topic:

```bash
ros2 topic epoch /imu/data
```

#### Data collection

Start the left camera node.

```bash
ros2 launch pylon_ros2_camera_wrapper 1_pylon_ros2_camera.launch.py
```

Start the right camera node.

```bash
ros2 launch pylon_ros2_camera_wrapper 2_pylon_ros2_camera.launch.py
```

Record camera data packets.

```bash
cd /data/bagfiles 
ros2 bag record /my_camera_1/pylon_ros2_camera_node/image_raw \
                /my_camera_2/pylon_ros2_camera_node/image_raw

```

```bash
ros2 bag record /my_camera_1/pylon_ros2_camera_node/image_raw \
                /my_camera_2/pylon_ros2_camera_node/image_raw \
                /imu/data /filter/positionlla
```

It is recommended to use the method described in the "multi-cameras" section.

Start the LiDAR node.

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

Record LiDAR data packets.

```bash
ros2 bag record /livox/lidar
```

Start the IMU node.

```bash
usermod -aG dialout root
chmod 777 /dev/ttyUSB0
ros2 launch bluespace_ai_xsens_mti_driver xsens_mti_node.launch.py
```

Record IMU data packets.

```bash
ros2 bag record /imu/data
```

Record GPS data packets.
```bash
ros2 bag record /gnss
```

#### Data visualization.

*Data visualization is only supported for playback of camera-recorded data packets.*

Execute a specific data packet.

```bash
ros2 bag play *.db3 --loop  # The `--loop` option is used to specify looping playback.
```

Start RViz for visualization.

```bash
ros2 run rviz2 rviz2

# You may need to execute the following command on the local host to grant permissions to the X server.
xhost +
```

Displaying data packet information.

```bash
ros2 bag info *.db3  
```

#### Data decompression.

Install [GitHub - Kyungpyo-Kim/ROS2BagFileParsing: ROS2 Bag file parsing](https://github.com/Kyungpyo-Kim/ROS2BagFileParsing) first.

You can use the following command for decompression:

```bash
source /ROS2BagFileParsing/dev_ws/src/ros2bag_parser/install/setup.bash
# alias DC

ros2 run ros2bag_parser ros2bag_parser_node
```


```
Usage: 
/ROS2BagFileParsing/dev_ws/src/ros2bag_parser/install/ros2bag_parser/lib/ros2bag_parser/ros2bag_parser_node \
<db3 file path> <output dir>
```

**It is important to note that when decompressing camera data packets, it is required to have a "data" folder specified in the target directory. However, for the IMU-generated CSV files, they do not need to be placed within the "data" folder.**

#### Camera calibration.

##### Method One

Start the corresponding Docker container.

```bash
docker stop wonderful_turing
docker start wonderful_turing
```

Interactively execute Docker commands.

```bash
docker exec -it wonderful_turing bash
```

And

```bash
source /catkin_ws/devel/setup.bash

rosrun kalibr kalibr_calibrate_cameras \
    --target /catkin_ws/src/target.yaml \
    --models pinhole-equi pinhole-equi \
    --topics /my_camera_1/pylon_ros2_camera_node/image_raw /my_camera_2/pylon_ros2_camera_node/image_raw \
    --bag /path/to/your/bag \
    --bag-freq 4.0 \
    --show-extraction
```



The path after "bag" needs to be replaced with your own path. If you are using ROS2 recording, you need to use rosbags-convert to convert the .db3 file to a .bag file (you can refer to https://github.com/ethz-asl/kalibr/wiki/ROS2-Calibration-Using-Kalibr for more details). The specific command is:

```bash
rosbags-convert <ros2_bag_folder> --dst calib_01.bag
```


##### Method Two

Run the following command directly in the collect_data container:

```bash
# Monocular.
ros2 run camera_calibration cameracalibrator --size 10x10 --square 0.06 --ros-args --remap image:=/my_camera_2/pylon_ros2_camera_node/image_raw --remap camera:=/my_camera_2/pylon_ros2_camera_node

# Stereo.
ros2 run camera_calibration cameracalibrator --size 10x10 --square 0.06 --ros-args --remap right:=/my_camera_2/pylon_ros2_camera_node/image_raw --remap left:=/my_camera_1/pylon_ros2_camera_node/image_raw --remap left_camera:=/my_camera_1/pylon_ros2_camera_node --remap right_camera:=/my_camera_2/pylon_ros2_camera_node
```

Record a video in real-time using a chessboard grid. Once all the progress bars turn green, proceed with the calibration.


#### Camera time synchronization.

**It is recommended to follow the method described in the "multi-cameras" section, which does not require executing the following steps.**

Execute the following commands sequentially to synchronize the camera timestamps with the host computer, record the data packets, and obtain the image titles as timestamps after decompression:

```bash
ros2 service call /my_camera_1/pylon_ros2_camera_node/enable_ptp std_srvs/srv/SetBool "{data: true}"

ros2 service call /my_camera_1/pylon_ros2_camera_node/set_periodic_signal_period pylon_ros2_camera_interfaces/srv/SetFloatValue "{value: 50000}"

ros2 service call /my_camera_1/pylon_ros2_camera_node/set_periodic_signal_delay pylon_ros2_camera_interfaces/srv/SetFloatValue "{value: 0}"

ros2 service call /my_camera_1/pylon_ros2_camera_node/set_trigger_selector pylon_ros2_camera_interfaces/srv/SetIntegerValue "{value: 0}"

ros2 service call /my_camera_1/pylon_ros2_camera_node/set_trigger_mode std_srvs/srv/SetBool "{data: true}"

ros2 service call /my_camera_1/pylon_ros2_camera_node/set_trigger_source pylon_ros2_camera_interfaces/srv/SetIntegerValue "{value: 5}"



ros2 service call /my_camera_2/pylon_ros2_camera_node/enable_ptp std_srvs/srv/SetBool "{data: true}"

ros2 service call /my_camera_2/pylon_ros2_camera_node/set_periodic_signal_period pylon_ros2_camera_interfaces/srv/SetFloatValue "{value: 50000}"

ros2 service call /my_camera_2/pylon_ros2_camera_node/set_periodic_signal_delay pylon_ros2_camera_interfaces/srv/SetFloatValue "{value: 0}"

ros2 service call /my_camera_2/pylon_ros2_camera_node/set_trigger_selector pylon_ros2_camera_interfaces/srv/SetIntegerValue "{value: 0}"

ros2 service call /my_camera_2/pylon_ros2_camera_node/set_trigger_mode std_srvs/srv/SetBool "{data: true}"

ros2 service call /my_camera_2/pylon_ros2_camera_node/set_trigger_source pylon_ros2_camera_interfaces/srv/SetIntegerValue "{value: 5}"

# alias PTP
```

```bash
ros2 run rqt_image_view rqt_image_view
```

