<a id="top"></a>

------------
## Contents

1. [Authors](#1.0)
2. [Required packages - Kinetic Version](#2.0)
3. [Easy install](#3.0)
4. [Run GGCNN in Gazebo and RVIZ](#4.0)

------------
<a name="1.0"></a>
## 1.0 - Authors
[Back to the top](#top)

- M.Sc. Caio Viturino* - [[Lattes](http://lattes.cnpq.br/4355017524299952)] [[Linkedin](https://www.linkedin.com/in/engcaiobarros/)] - engcaiobarros@gmail.com
- M.Sc. Kleber de Lima Santana Filho** - [[Lattes](http://lattes.cnpq.br/3942046874020315)] [[Linkedin](https://www.linkedin.com/in/engkleberfilho/)] - engkleberf@gmail.com
- M.Sc. Daniel M. de Oliveira* - [[Linkedin](https://www.linkedin.com/in/daniel-moura-de-oliveira-9b6754120/)] - danielmoura@ufba.br 
- Prof. Dr. André Gustavo Scolari Conceição* - [[Lattes](http://lattes.cnpq.br/6840685961007897)] - andre.gustavo@ufba.br

*LaR - Laboratório de Robótica, Departamento de Engenharia Elétrica e de Computação, Universidade Federal da Bahia, Salvador, Brasil

**PPGM - Programa de Pós-Graduação em Mecatrônica, Universidade Federal da Bahia, Salvador, Brasil.

------------
<a name="2.0"></a>
## 2.0 - Required packages - Kinetic Version
[Back to the top](#top)

This code was developed with Python 2.7 on Ubuntu 16.04 with ROS Kinetic.

**_It is recommended to follow the EASY INSTALL step below_**

- [Realsense Gazebo Plugin](https://github.com/pal-robotics/realsense_gazebo_plugin)
- [Realsense-ros](https://github.com/IntelRealSense/realsense-ros) Release version 2.2.11
- [Librealsense](https://github.com/IntelRealSense/librealsense) Release version 2.31.0 - Install from source
- [Moveit Kinetic](https://moveit.ros.org/install/)
- [Moveit Python](https://github.com/mikeferguson/moveit_python)
- [Robotiq Gripper](https://github.com/crigroup/robotiq)
- [Universal Robot](https://github.com/ros-industrial/universal_robot)
- [ur_modern_driver](https://github.com/ros-industrial/ur_modern_driver)
- [Gluoncv](https://github.com/dmlc/gluon-cv)
- [Opencv](https://github.com/opencv/opencv)
- [Mxnet](https://mxnet.apache.org/) Install Mxnet for your CUDA version.

> **_NOTE:_**  This package should be placed into your src folder. Please open an issue if you find any problem related to this package.

------------
<a name="3.0"></a>
## 3.0 - Easy install
[Back to the top](#top)

In order to install all the required packages easily, create a new catkin workspace
```bash
mkdir -p ~/catkin_ws_new/src
```

Clone this repository into the src folder
```bash
cd ~/catkin_ws_new/src
git clone https://github.com/caiobarrosv/selective_grasping
```

Run the install.sh file
```bash
cd ~/catkin_ws_new/src/selective_grasping/install
./install.sh
```
------------
<a name="4.0"></a>
## 4.0 - Run the experiments in Gazebo
[Back to the top](#top)

> **THE GAZEBO SIMULATION IS NOT WORKING PROPERLY. THE GRASPS ARE UNSTABLE AND NEED TO BE FIXED. THE SIMULATION IS USED ONLY TO RUN EXPERIMENTS PRIOR TO THE REAL WORLD APPLICATION TO AVOID ACCIDENTS**

Please follow each following steps:

### 4.1 - Launch Gazebo:

```bash
roslaunch selective_grasping gazebo_ur5.launch
```

### 4.2 - Run the UR5 control node 
Press enter after the following message appears and jump to the step 4.3:
"==== Press enter to move the robot to the 'depth cam shot' position!"
```bash
rosrun selective_grasping ur5_open_loop.py --gazebo
```

### 4.3 - Run the SSD node
```bash
rosrun selective_grasping detect.py --gazebo
```

### 4.4 - Run the GG-CNN node
Press enter after the following message appears and jump to the step 4.5:
"Press enter to start the GGCNN"
```bash
rosrun selective_grasping run_ggcnn.py --gazebo
```

### 4.5 - Spawn the objects in the workspace
```bash
rosrun selective_grasping spawn_objects.py
```

### 4.6 - UR5 control node
After running the GG-CNN node you are able to move the robot and perform the grasp using the terminal provided by step 4.2



### 4.7 - Change the Gazebo properties (OPTIONAL)
It will speed up your Gazebo simulation a little bit :)
```bash
rosrun selective_grasping change_gazebo_properties.py
```

### 4.8 - Visualize the images published by the GG-CNN
You might want to see the grasp or any other image. In order to do that, you can use the rqt_image_view.
```bash
rosrun rqt_image_view
```

### 4.9 - Visualize depth cloud in RVIZ
If you want to visualize the data being published by the Intel Realsense D435 please run the following node:
```bash
rosrun selective_grasping rviz_ur5.launch
```