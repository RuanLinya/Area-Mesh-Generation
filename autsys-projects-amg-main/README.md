# autsys-projects-amg
## Challenge 3 - Area Mesh Generation (Vision)
### Table of contents
1. Project Description
2. Members
3. Installation and Build
4. Run the Simulation
5. Responsibilities 
6. Milestones
7. Demo
8. Architecture Overview and Interfaces
9. Reference

### Project Description

This project explore the process of 3D slightly hilly terrain containing a polygon of n corners reconstruction using UAVs with down-pointing stereo-camera. A mesh reconstruction of area is generated from raw images after UAVs scans this area. The core modules are trajectory planning, feature matching, point cloud generation and merging, colored mesh generation.

![image](https://github.com/TUM-AAS/autsys-projects-amg/blob/main/image/enviroment.png)


The main tasks of this project are as follows:

• trajectory planning to cover terrain

• perform feature-matching from stereo images

• point cloud from features generation

• a mesh of the ground surface generation

### Members

| Name | TUM-ID |
|------| ------ |
| Hang Li | ge23zop |
| Tiantian Wei | ge49rep |
| Ziting Huang | ge95qus |
| Linya Ruan | ge35wod |

### Installation and Build

Ubuntu version:
```
Ubuntu 20.04 LTS 
```
Before building, you may need to install OpenCV and PCL.

Install OpenCV:
```
sudo apt update
sudo apt install libopencv-dev python3-opencv
```
Install PCL:
```
sudo apt install libpcl-dev
```
Now you can start to build. Clone the repository and cd to '../catkin_ws'. In a terminal, run:
```
git submodule update --init --recursive
```
```
catkin build
```

### Run the Simulation:

After building, you can start to run the code.

In '../catkin_ws', source the environment in terminal:
```
source devel/setup.bash
```
Launch the simulation:
```
roslaunch simulation simulation.launch
```

Now the drone starts to fly on predifined trajectory. In Rviz window, you will see, that the generated point cloud are merging. In a few minutes you are firstly expected to see this PCL viewer window, which shows two Meshs with RGB information generated from Greedy triangle and Poisson reconstruction algorithm. 

![image](https://github.com/TUM-AAS/autsys-projects-amg/blob/main/image/RGB.png)

Then wait a few minutes, you are expected to see another PCL viewer window, which shows four meshs. On the upper part are two meshs without RGB color. On the lower side are two meshs with color information based on the height. The color change from blue to red according to the height.

![image](https://github.com/TUM-AAS/autsys-projects-amg/blob/main/image/height.png)
### Responsibilities 

- trajectory planning to cover terrain: Ziting Huang, Linya Ruan
- perform feature-matching from stereo images: Ziting Huang, Linya Ruan
- point cloud from features generation: Hang Li, Tiantian Wei
- a mesh of the ground surface generation: Hang Li, Tiantian Wei
- Debug: all

### Milestones

- phase 1: trajectory planning to cover terrain (13/01/2023-27/01/2023)
- phase 2: perform feature-matching from stereo images (28/01/2023-12/02/2023)
- phase 3: point cloud from features generation (12/02/2023-01/03/2023)
- phase 4: a mesh of the ground surface generation (01/03/2023-17/03/2023)
- phase 5: Report (17/03/2023-22/02/2023)

### Demo
- Area Mesh Generation:
https://youtu.be/wqpocfeieUQ

- Unity environments with distinctive features of the terrain:
https://youtu.be/jiScXNfc3rE

### Architecture Overview and Interfaces
![image](https://github.com/TUM-AAS/autsys-projects-amg/blob/main/image/rosgraph.png)

### Reference

1. Mohsan, S.A.H., Othman, N.Q.H., Li, Y. et al. Unmanned aerial vehicles (UAVs): practical aspects, applications, open challenges, security issues, and future trends. Intel Serv Robotics 16, 109–137 (2023). https://doi.org/10.1007/s11370-022-00452-4
2. http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
3. https://johnwlambert.github.io/stereo/
4. http://wiki.ros.org/stereoimageproc
5. http://wiki.ros.org/rviz (Accessed on Mar. 21, 2023)
6. https://diglib.eg.org/handle/10.2312/SGP.SGP06.061-070
7. https://pointclouds.org/documentation/tutorials/index.html
8. http://t.csdn.cn/YpuzY
9. http://t.csdn.cn/Dyq1a

