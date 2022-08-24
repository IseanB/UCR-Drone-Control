# UCR Drone Control

This repository combines a trajectory planning, communication protocol, and robotics libraries to autonomously control drone(s). Our code is built with the [ROS (Melodic)](http://wiki.ros.org/melodic) and [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) frameworks to develop drone control software. The [MAV Trajectory Generation](https://github.com/ethz-asl/mav_trajectory_generation) library generates an optimized, minimum snap, path for our drone(s) to follow. Finally, [MAVLink (MAVROS)](http://wiki.ros.org/mavros) is our primary messaging protocol, interfacing with the drone(s).

---

**Advisors:** Hanzhe Teng

**Developers:** Isean Bhanot

---
## Installation 
  Tip: Only use ```catkin build```, never use ```catkin_make```.
  1. Install VMWare and set up a disc image of Ubuntu 18.04 LTS.[^11][^12] 
     - The rest of these instructions take place in the Ubuntu environment.
     - Disable "Accelerate 3D graphics" setting under *Virtual Machine Settings* -> *Display*.
  2. Install Git on VMWare and set up SSH keys.[^14]
     - Use ```sudo apt install git``` to install Git.
  3. Follow the [Ubuntu Development Enviorment](https://wiki.hanzheteng.com/quadrotor/px4#ubuntu-development-environment) and [Gazebo SITL Simulation](https://wiki.hanzheteng.com/quadrotor/px4#gazebo-sitl-simulation) setup. 
     - Install PX4 Related Dependencies. [^1]
       - Once *Firmware* folder cloned, use the latest stable version. i.e. Run ```git checkout tags/v1.13.0``` in *Firmware*.
       - Delete *Firmware* folder after completing steps. 
     - Install ROS Melodic. [^2]
       - Full Desktop Installation
     - Install Simulation Common Dependencies. [^3]
       - Run all code, except line 12, line by line in the terminal.
       - Running pyulog install command is not necessary.
     - Build Quadrotor Model & Setup Gazebo[^10]
       - Before building quadcopter models, update Pillow[^6] and GStreamer[^7]
     - Install MAVROS, MAVLink, & Setup Workspace [^4]
       - Source Installation (Released/Stable)
  4. Install [MAV Trajectory Generation](https://github.com/ethz-asl/mav_trajectory_generation) Package [^13]
     - When installing additional system dependencies, replace ```indigo``` with ```melodic```.
     - Replace ```catkin config --merge-devel``` with ```catkin config --link-devel```
  
  5. Install QGroundContol [^5]
     - Install Linux/Ubuntu version.
  
  6. Create a ROS package "drone_control"
  ```
  cd ~/[Workspace Name]/src
  git clone --recursive git@github.com:IseanB/UCR-Drone-Control.git
  mv UCR-Drone-Control drone_control
  cd drone_control
  cd ~/[Workspace Name]
  catkin build
  . ~/[Workspace Name]/devel/setup.bash
  ```
  
### Installation Bug Fixes:
1. Pillow Build Error [^6]
2. GStreamer Error [^7]
3. Gazebo/Rendering Error [^9]
4. Symforce Error (Look above, under *Install PX4 Related Dependencies* for fix.)

---
## Documentation

The ***drone_control*** package is structured into three main folders: the src, helper, and the test folder. Below are quick summaries of each folder's purpose.
### src
The src folder stores all the control code used to create the offboard node. Currently, the file offboard.cpp optimizes the path followed, controls the state of the drone, and sends control signals to the drone. 

The control begins with initializing all the variables(position, velocity, droneState, subscriber nodes, publisher nodes...) used, calculating a hard-coded path, and then connecting to the drone. Next, three loops represent three primary states; [LIFTING_OFF](https://github.com/IseanB/UCR-Drone-Control/blob/7b9c39f9257bec2359fcdae1e4ba6a6e2a96feaa/src/offboard.cpp#L146-L174), [IN_TRANSIT](https://github.com/IseanB/UCR-Drone-Control/blob/7b9c39f9257bec2359fcdae1e4ba6a6e2a96feaa/src/offboard.cpp#L177-L223), and [LANDING](https://github.com/IseanB/UCR-Drone-Control/blob/7b9c39f9257bec2359fcdae1e4ba6a6e2a96feaa/src/offboard.cpp#L227-L254); the drone could be in.

Each loop contains unique control software used during that state of the drone. Control of the drone is passed from one loop to another based on the state the drone is in. See the offboard.cpp file for more info about how the control works.
### helper
The helper folder stores any mathematical computations or data conversions needed. This allows for testing and easy extensibility for future computations/conversions. Some of the functions written are the isStationary(...), isFlat(...), reachedLocation(...), and segmentToPoint(...).
### test
The test folder contains tests for the helper functions in the helper folder. Some of the functions tested are mathematical computations, such as isStationary(...), while others are data conversions like in the segmentToPoint(...) function. GoogleTest is the test framework used.
### launch
The launch folder contains the setup of the Gazebo simulation and MAVROS nodes. The fourDrones and oneDrone.launch are example launch files used for testing the single/multi control structure.
### msg
The msg folder contains the custom messages that are communicated between the individual drone nodes and the multi control node.
---
## Usage

### Launching World in Gazebo
Tip: Before launch run ```echo "export SVGA_VGPU10=0" >> ~/.bashrc``` and ```source ~/.bashrc```, to prevent VMWare REST Error below. [^8] 

```roslaunch drone_control fourDronesNodes.launch```

### Running drone# node(single drone control)
```rosrun drone_control single_control #(0-3)```

### Running multi_control node(multi drone control)
```rosrun drone_control single_control```

### Running Google Tests(Optional)
```rosrun drone_control simple_mov_test```

---
## Tools
  - C++
  - Catkin(Modified CMake)
  - Gazebo
  - GoogleTest
  - Git
  - Linux
  - VSCode
  - VMWare

 ## Libraries
  - [MAV Trajectory Generation](https://github.com/ethz-asl/mav_trajectory_generation#bibliography)
  - [MAVLink: MAVROS](https://github.com/mavlink/mavros)
  - [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)
  - [ROS(Melodic)](http://wiki.ros.org/melodic)

---

### References
  - [Hanzhe Teng Wiki](https://wiki.hanzheteng.com/)
  - [MAVROS API](http://wiki.ros.org/mavros)
  - [MAV Trajectory Generation API](https://mav-trajectory-generation.readthedocs.io/en/latest/api/library_root.html#full-api)
  - [ROS/Gazebo](https://docs.px4.io/main/en/simulation/ros_interface.html)
  - [Sample Code 1](https://automaticaddison.com/how-to-move-the-turtlesim-robot-to-goal-locations-ros/)
  - [Sample Code 2](https://docs.px4.io/v1.12/en/ros/mavros_offboard.html)

  [^note]:
  [^1]:https://dev.px4.io/v1.11_noredirect/en/setup/dev_env_linux_ubuntu.html (PX4 Install in Root)
  [^2]:http://wiki.ros.org/melodic/Installation/Ubuntu (ROS - Desktop Full Installation)
  [^3]:https://github.com/PX4/PX4-Devguide/blob/master/build_scripts/ubuntu_sim_common_deps.sh (Simulation - Run code after line 12 in terminal)
  [^4]:https://docs.px4.io/v1.12/en/ros/mavros_installation.html (MAVROS - Source Installation, Install Released/Stable MAVROS, use *sudo* when installing GeographicLib)
  [^5]:https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html (QGroundControl)
  [^6]:https://pillow.readthedocs.io/en/stable/installation.html (Pillow)
  [^7]:https://gstreamer.freedesktop.org/documentation/installing/on-linux.html?gi-language=c (GStreamer)
  [^8]:https://answers.gazebosim.org//question/25030/gazebo-error-restcc205-error-in-rest-request/ (VMWare) 
  [^9]:https://answers.gazebosim.org//question/13214/virtual-machine-not-launching-gazebo/ (Gazebo)
  [^10]:https://wiki.hanzheteng.com/quadrotor/px4#gazebo-sitl-simulation (Gazebo)
  [^11]:https://customerconnect.vmware.com/en/downloads/info/slug/desktop_end_user_computing/vmware_workstation_pro/16_0 (VMWare)
  [^12]:https://releases.ubuntu.com/18.04/ (Ubuntu)
  [^13]:https://github.com/ethz-asl/mav_trajectory_generation#installation-instructions-ubuntu (MAV Trajcetory Generation)
  [^14]:https://www.digitalocean.com/community/tutorials/how-to-configure-ssh-key-based-authentication-on-a-linux-server (SSH Keys)
