# UCR Drone Control

## Installation 
  Tip: Only use ```catkin build```, never use ```catkin_make```.
  1. Install PX4 Related Dependencies. [^1]
 
  2. Install ROS Melodic. [^2]

  3. Install Simulation Common Dependencies. [^3]

  4. Build Quadrotor Model & Setup Gazebo[^10]
  
  5. Install MAVROS & Setup Workspace [^4]
  
  6. Install QGroundContol [^5]
  
  7. Create a ROS package "simple_movements"
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
  - [Hanzhe Teng Wiki](https://wiki.hanzheteng.com/)
  - [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)
  - [Gazebo](https://docs.px4.io/main/en/simulation/ros_interface.html)
  - [MAVLink: MAVROS](https://github.com/mavlink/mavros)
  - [MAVROS API](http://wiki.ros.org/mavros)
  - [MAV Trajectory Generation](https://github.com/ethz-asl/mav_trajectory_generation#bibliography)
  - [Sample Code 1](https://automaticaddison.com/how-to-move-the-turtlesim-robot-to-goal-locations-ros/)
  - [Sample Code 2](https://docs.px4.io/v1.12/en/ros/mavros_offboard.html)
 
 ## Tools
  - Gazebo
  - ROS(Melodic)
  - QGroundControl
  - VMWare
  - Linux
  - C++
  - Git
 

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
