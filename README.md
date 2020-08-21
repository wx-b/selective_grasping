
------------

<a id="top"></a>
### Contents
1. [Authors](#1.0)
2. [Required packages - Kinetic Version](#3.0)
3. [Run GGCNN in Gazebo and RVIZ](#4.0)

------------
<a name="1.0"></a>
### 1.0 - Authors

- M.Sc. Caio Viturino* - [[Lattes](http://lattes.cnpq.br/4355017524299952)] [[Linkedin](https://www.linkedin.com/in/engcaiobarros/)] - engcaiobarros@gmail.com
- M.Sc. Kleber de Lima Santana Filho** - [[Lattes](http://lattes.cnpq.br/3942046874020315)] [[Linkedin](https://www.linkedin.com/in/engkleberfilho/)] - engkleberf@gmail.com
- M.Sc. Daniel M. de Oliveira* - [[Linkedin](https://www.linkedin.com/in/daniel-moura-de-oliveira-9b6754120/)] - danielmoura@ufba.br 
- Prof. Dr. André Gustavo Scolari Conceição* - [[Lattes](http://lattes.cnpq.br/6840685961007897)] - andre.gustavo@ufba.br

*LaR - Laboratório de Robótica, Departamento de Engenharia Elétrica e de Computação, Universidade Federal da Bahia, Salvador, Brasil

**PPGM - Programa de Pós-Graduação em Mecatrônica, Universidade Federal da Bahia, Salvador, Brasil.

<a name="2.0"></a>
### 2.0 - Required packages - Kinetic Version

This code was developed with Python 2.7 on Ubuntu 16.04 with ROS Kinetic.

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

#### Easy install

In order to install all the required packages easily, create a new catkin workspace
```bash
mkdir -p ~/catkin_ws_new/src
```

Clone this repository into the src folder
```bash
cd ~/catkin_ws_new/src
git clone https://github.com/lar-deeufba/ssggcnn_ur5_grasping
```

Run the install.sh file
```bash
cd ~/catkin_ws_new/src/ssggcnn_ur5_grasping/install
sudo chmod +x ./install.sh
./install.sh
```

#### This repository also need the SSD512 implementation created by [czrcbl](https://github.com/czrcbl). Please follow the next procedures provided by the author.

Install bboxes before continuing. You can install directly from `github`:
```bash
pip install git+https://github.com/czrcbl/bboxes
```

Or you can clone the repository and install on editable mode:
```bash
git clone https://github.com/czrcbl/bboxes
cd bboxes
git install -e .
```

Download the [model.params](https://drive.google.com/open?id=1NdoknYGaaReDcmeLYm6bUe6ksv84ueOo) in the following link and move it to the `scripts/detection_pkg` folder.

<a name="3.0"></a>
### 3.0 - Run SSGG-CNN in Gazebo
Please follow each following steps:

#### 3.1 - Launch Gazebo:
```bash
roslaunch ssggcnn_ur5_grasping gazebo_ur5.launch
```

#### 3.2 - Run the UR5 control node 
Press enter after the following message appears and jump to the step 3.3:
"==== Press enter to move the robot to the 'depth cam shot' position!"
```bash
rosrun ssggcnn_ur5_grasping ur5_open_loop.py --gazebo
```

#### 3.3 - Run the SSD node
```bash
rosrun ssggcnn_ur5_grasping main.py
```

#### 3.4 - Run the GG-CNN node
Press enter after the following message appears and jump to the step 3.5:
"Press enter to start the GGCNN"
```bash
rosrun ssggcnn_ur5_grasping run_ggcnn.py --ssggcnn
```

#### 3.5 - Spawn the objects in the workspace
```bash
rosrun ssggcnn_ur5_grasping spawn_objects.py
```

#### 3.6 - UR5 control node
After running the GG-CNN node you are able to move the robot and perform the grasp.
Press enter to complete each related task specified in ur5_open_loop.py

#### 3.7 - Change the Gazebo properties (OPTIONAL)
It will speed up your Gazebo simulation a little bit :)
```bash
rosrun ssggcnn_ur5_grasping change_gazebo_properties.py
```

#### 3.8 - Visualize the images published by the GG-CNN
You might want to see the grasp or any other image. In order to do that, you can use the rqt_image_view.
```bash
rosrun rqt_image_view
```

#### 3.9 - Visualize depth cloud in RVIZ
If you want to visualize the data being published by the Intel Realsense D435 please run the following node:
```bash
rosrun ssggcnn_ur5_grasping rviz_ur5.launch
```
