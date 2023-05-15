
# Area Mesh Generation (Vision)


### Project Description

This project explore the process of 3D slightly hilly terrain containing a polygon of n corners reconstruction using UAVs with down-pointing stereo-camera. A mesh reconstruction of area is generated from raw images after UAVs scans this area. The core modules are trajectory planning, feature matching, point cloud generation and merging, colored mesh generation.

<img width="264" alt="image" src="https://github.com/RuanLinya/Area-Mesh-Generation/assets/133128176/377febb3-3f39-46f4-8bf9-7476c841f37b">

The main tasks of this project are as follows:

• trajectory planning to cover terrain

• perform feature-matching from stereo images

• point cloud from features generation

• a mesh of the ground surface generation


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

<img width="343" alt="image" src="https://github.com/RuanLinya/Area-Mesh-Generation/assets/133128176/7031e2a1-ad47-405c-a8b4-c28b48967516">

Then wait a few minutes, you are expected to see another PCL viewer window, which shows four meshs. On the upper part are two meshs without RGB color. On the lower side are two meshs with color information based on the height. The color change from blue to red according to the height.

<img width="310" alt="image" src="https://github.com/RuanLinya/Area-Mesh-Generation/assets/133128176/b5889d38-0322-4d4d-83ba-d0ea12c42229">

### Demo
- Area Mesh Generation:
https://youtu.be/wqpocfeieUQ

- Unity environments with distinctive features of the terrain:
https://youtu.be/jiScXNfc3rE

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

