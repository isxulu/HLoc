# Sample

## 数据采集

### docker(env)

通过镜像创建容器：

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

启动容器：

```bash
docker start collect
```

运行docker镜像：

```bash
docker exec -it collect bash  # 其中collect_data为容器名
```

### ROS

#### 基本命令

配置工作空间：

```bash
source /ws/install/setup.bash
```

查看当前所有的话题：

```bash
ros2 topic list
```

查看某个话题的信息发送的频率：

```bash
ros2 topic hz /imu/data  
```

打印某个话题的内容：

```bash
ros2 topic epoch /imu/data
```

#### 数据采集

启动左相机节点：

```bash
ros2 launch pylon_ros2_camera_wrapper 1_pylon_ros2_camera.launch.py
```

启动右相机节点：

```bash
ros2 launch pylon_ros2_camera_wrapper 2_pylon_ros2_camera.launch.py
```

录制相机数据包：

```bash
cd /data/bagfiles  # 将录制文件存在/data/bagfiles目录下
ros2 bag record /my_camera_1/pylon_ros2_camera_node/image_raw \
                /my_camera_2/pylon_ros2_camera_node/image_raw

# 也可以分开采集，启动两个终端
```

```bash
ros2 bag record /my_camera_1/pylon_ros2_camera_node/image_raw \
                /my_camera_2/pylon_ros2_camera_node/image_raw \
                /imu/data /filter/positionlla
```

更推荐采用`muti-cameras`中方法.

启动激光雷达节点：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

录制激光数据包：

```bash
ros2 bag record /livox/lidar
```

启动IMU节点：

```bash
usermod -aG dialout root
chmod 777 /dev/ttyUSB0
ros2 launch bluespace_ai_xsens_mti_driver xsens_mti_node.launch.py
```

录制IMU数据包：

```bash
ros2 bag record /imu/data
```

录制GPS数据包:
```bash
ros2 bag record /gnss
```

#### 数据可视化

*可视化仅支持回放相机录制的数据包。*

执行某个数据包：

```bash
ros2 bag play *.db3 --loop  # --loop选项用于指定循环播放
```

启动rviz进行可视化：

```bash
ros2 run rviz2 rviz2

# 可能需要在本地主机执行下方的命令赋予X server的权限
xhost +
```

展示数据包信息：

```bash
ros2 bag info *.db3  # 名称依据具体的数据库而定
```

#### 数据解压

需要安装[GitHub - Kyungpyo-Kim/ROS2BagFileParsing: ROS2 Bag file parsing](https://github.com/Kyungpyo-Kim/ROS2BagFileParsing).

解压时可以执行以下命令：

```bash
source /ROS2BagFileParsing/dev_ws/src/ros2bag_parser/install/setup.bash
# alias DC

ros2 run ros2bag_parser ros2bag_parser_node
```

shell中会输出解压软件的用法：

```
Usage: 
/ROS2BagFileParsing/dev_ws/src/ros2bag_parser/install/ros2bag_parser/lib/ros2bag_parser/ros2bag_parser_node \
<db3 file path> <output dir>
```

只需要将其中的数据源和目录路径替换为自己的即可。

**特别需要注意的是，在解压相机数据包时，要求指定的目录下必须有data文件夹，而IMU生成的csv文件不需要在data文件中。**

#### 相机标定

##### 方法一

启动对应的docker:

```bash
docker stop wonderful_turing
docker start wonderful_turing
```

交互执行docker:

```bash
docker exec -it wonderful_turing bash
```

随后依次执行：

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



其中bag后的路径需要替换为自己的路径，如果采用ros2录制需要使用`rosbags-convert`将.db3转换为.bag文件(具体可以参考https://github.com/ethz-asl/kalibr/wiki/ROS2-Calibration-Using-Kalibr)，具体命令是：

```bash
rosbags-convert <ros2_bag_folder> --dst calib_01.bag
```

其中`.bag`的文件名可以自己指定。


##### 方法二

直接在collect_data容器中，运行：

```bash
# 单目
ros2 run camera_calibration cameracalibrator --size 10x10 --square 0.06 --ros-args --remap image:=/my_camera_2/pylon_ros2_camera_node/image_raw --remap camera:=/my_camera_2/pylon_ros2_camera_node

# 双目
ros2 run camera_calibration cameracalibrator --size 10x10 --square 0.06 --ros-args --remap right:=/my_camera_2/pylon_ros2_camera_node/image_raw --remap left:=/my_camera_1/pylon_ros2_camera_node/image_raw --remap left_camera:=/my_camera_1/pylon_ros2_camera_node --remap right_camera:=/my_camera_2/pylon_ros2_camera_node
```

使用棋盘格实时录制视频，待进度条都变为绿色后，进行标定。


#### 相机时间同步

**更推荐采取`multi-cameras`中的方法,无需执行下面的步骤.**

依次执行下面的命令，可以将相机的时间戳与电脑主机同步，录制包并解压后得到的图像标题就是时间戳。

IMU不需要同步。

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

