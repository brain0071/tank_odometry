```javascript


A.Apriltag 里程计

1.安装apriltag 
注意:确保librealsense安装在"usr/local/lib"中,可以尝试先安装librealsense,然后安装apriltag,再重新编译apriltag
https://github.com/IntelRealSense/librealsense/tree/R/250/examples/pose-apriltag

2.安装librealsense(2.50.0)
https://github.com/IntelRealSense/librealsense/blob/R/250/doc/installation.md
在编译中加入 "-DCMAKE_PREFIX_PATH=~/apriltag"

3.编译pose-apriltag,librealsense的example中的pose-apriltag的ros封装;
4.执行tank-Odometry节点
注意:在Ocean rotor机器人中,坐标系使用Xsens的ENU坐标系.
(1)t265 camera Coordinate system 
https://dev.intelrealsense.com/docs/intel-realsensetm-visual-slam-and-the-t265-tracking-camera
(2)Xsens Coordinate system  
https://mtidocs.movella.com/getting-started-2

1.apriltag位置代码更新，去掉了ekf(更新频率为5hz)
ekf项目中，使用了带有不同高度的标签，我们的水池假设所有标签z轴位置在同一水平面，所以水池中不能实现z轴定位。另外由于单个apriltag的定位精度比较准确，ekf比较冗余。
目前更新的代码中，根据标签在水池坐标系中的位置、广角相机坐标系中检测到的标签位置转换结果，得到相机相对于水池坐标系原点的一组绝对坐标，然后求平均值得到相机在水池坐标系中的位置。

注意:实验中需要将t265相机向下平行放置，另外水池中标签的坐标轴需要与ENU坐标系一致。

(1)目前更新的代码使用了t265官方代码中的apriltag库，所以应该比较可靠，目前测试结果比较稳定，位置和速度效果都很好，换算过程中使用了相机的内外参,要考虑水下畸变情况。

B.IMU积分速度里程计
1.安装mavros
https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation
注意:FCU中使用NED坐标系，Mavros转换为ENU坐标系

测试了使用IMU积分得到速度的方法，测试平台为pixhawk,在每个采样时刻将IMU得到的加速度结果进行累加，然后每3s进行一次reset。Imu数据频率(100hz)

检查IMU数据频率的命令:
"rostopic hz /mavros/imu/data"
修改pixhawk IMU频率(100hz)的命令:
"rosrun mavros mavcmd long 511 105 10000 0 0 0 0 0" /mavros/imu/data_raw
"rosrun mavros mavcmd long 511 31 10000 0 0 0 0 0 " /mavros/imu/data

目前得到的效果不是很好，在有翻转时候，重力加速度会出现比较大的假值，而且在速度较大的时候，速度信息也可能刚好被reset，在速度比较大的时候，IMU数据也可能不准。

**以上内容需要使用Motion Capture做精度测试对比。
**Pixhawk数据质量很差,需要进一步验证坐标系





```

