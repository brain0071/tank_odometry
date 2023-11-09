```javascript

1.安装apriltag 
https://github.com/IntelRealSense/librealsense/tree/R/250/examples/pose-apriltag

2.安装librealsense(2.50.0)
注意:确保librealsense安装在"usr/local/lib"中,可以尝试先安装librealsense,然后安装apriltag,再重新编译apriltag
https://github.com/IntelRealSense/librealsense/blob/R/250/doc/installation.md
在编译中加入 "-DCMAKE_PREFIX_PATH=~/apriltag"

3.编译pose-apriltag,librealsense的example中的pose-apriltag的ros封装;

4.执行tank-Odometry节点

5.安装pose-apriltag

注意:在Ocean rotor机器人中,坐标系使用Xsens的ENU坐标系.
(1)t265 camera Coordinate system 
https://dev.intelrealsense.com/docs/intel-realsensetm-visual-slam-and-the-t265-tracking-camera
(2)Xsens Coordinate system  
https://mtidocs.movella.com/getting-started-2

6.todo:将IMU数据提取出来做速度积分;对比apriltag结果做精度验证







```

