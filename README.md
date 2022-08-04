# UCR Drone Control


This repository uses various control and robotics libraries to autonomously control drone(s). Our code is built with the [ROS (Melodic)](http://wiki.ros.org/melodic) and [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) frameworks to develop drone control software. The [MAV Trajectory Generation](https://github.com/ethz-asl/mav_trajectory_generation) library generates an optimized, minimum snap, path for our drone(s) to follow. Finally, [MAVLink (MAVROS)](http://wiki.ros.org/mavros) is our primary messaging protocol, interfacing with the drone(s).

**Advisors:** Hanzhe Teng

**Developers:** Isean Bhanot

## Installation 
  Tip: Only use ```catkin build```, never use ```catkin_make```.
  1. Install VMWare and setup a disc image of Ubuntu 18.04 LTS.[^11][^12] 
     - The rest of these instructions take place in the Ubuntu enviorment.
  2. Follow the [Ubuntu Development Enviorment](https://wiki.hanzheteng.com/quadrotor/px4#ubuntu-development-environment) and [Gazebo SITL Simulation](https://wiki.hanzheteng.com/quadrotor/px4#gazebo-sitl-simulation) setup. 
     - Install PX4 Related Dependencies. [^1]
       - Delete PX4 folder after completing steps.
     - Install ROS Melodic. [^2]
       - Full Desktop Installation
     - Install Simulation Common Dependencies. [^3]
       - Run all code, execpt line 12, line by line in terminal.
     - Build Quadrotor Model & Setup Gazebo[^10]
     - Install MAVROS, MAVLink, & Setup Workspace [^4]
       - Source Installation (Released/Stable)
  
  3. Install QGroundContol [^5]
  
  4. Create a ROS package "simple_movements"
  ```
  cd ~/[Workspace Name]/src
  mkdir simple_movements
  cd simple_movements/
  git clone --recursive https://github.com/IseanB/UCR-Drone-Control.git
  cd ~/[Workspace Name]
  catkin build
  . ~/[Workspace Name]/devel/setup.bash
  ```
  
### Installation Bug Fixes:
1. Pillow Build Error [^6]
2. GStreamer Error [^7]
3. VMWare REST Error [^8]
4. Gazebo/Rendering Error [^9]
 
 ## Libraries / References
  - [MAV Trajectory Generation Library](https://github.com/ethz-asl/mav_trajectory_generation#bibliography)
  - [MAVLink: MAVROS Library](https://github.com/mavlink/mavros)
  - [PX4 Autopilot Library](https://github.com/PX4/PX4-Autopilot)
  - [ROS(Melodic) Library](http://wiki.ros.org/melodic)
  - [Hanzhe Teng Wiki](https://wiki.hanzheteng.com/)
  - [MAVROS API](http://wiki.ros.org/mavros)
  - [MAV Trajectory Generation API](https://mav-trajectory-generation.readthedocs.io/en/latest/api/library_root.html#full-api)
  - [ROS/Gazebo](https://docs.px4.io/main/en/simulation/ros_interface.html)
  - [Sample Code 1](https://automaticaddison.com/how-to-move-the-turtlesim-robot-to-goal-locations-ros/)
  - [Sample Code 2](https://docs.px4.io/v1.12/en/ros/mavros_offboard.html)
  

 ## Tools
  - C++
  - Gazebo
  - GoogleTest
  - Git
  - Linux
  - QGroundControl
  - VMWare

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
