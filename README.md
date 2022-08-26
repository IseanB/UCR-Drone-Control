# UCR Drone Control

This repository combines a trajectory planning, communication protocol, and robotics libraries to autonomously control multiple drones. Our code is built with the [ROS (Melodic)](http://wiki.ros.org/melodic) and [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) frameworks to develop drone control software. The [MAV Trajectory Generation](https://github.com/ethz-asl/mav_trajectory_generation) library generates an optimized, minimum snap, path for our drones to follow. Finally, [MAVLink (MAVROS)](http://wiki.ros.org/mavros) is our primary messaging protocol, interfacing with the drones.

---

**Advisors:** Hanzhe Teng

**Developers:** Isean Bhanot

---
## Installation 
  Tip: Only use ```catkin build```, never use ```catkin_make```.
  1. Install VMWare and set up a disc image of Ubuntu 18.04 LTS.[^11][^12] 
     - Disable "Accelerate 3D graphics" setting under *Virtual Machine Settings* -> *Display*.
     - The rest of these instructions take place in the Ubuntu environment.
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
## Usage
### Running Multi Drone Control

Once the installation process is complete, these terminal commands properly run the drone_control package. 
1. Run ```roslaunch drone_control fourDronesNodes.launch```.
   - Initializes simulated world, MAVROS nodes, and spawn the four drones.
2. Run ```rosrun drone_control single_control (0-3)``` with a different number in the range [0,3].
   - Initialize nodes for all four drones.
   - Run in the command in seperate terminals windows/tabs with a different number, for all the drones. 
3. Run ```rosrun drone_control multi_control```.
   - Runs the multi drone control code.
   - To prevent the VMWare REST Error, run ```echo "export SVGA_VGPU10=0" >> ~/.bashrc``` and ```source ~/.bashrc```.[^8]

### Commands

#### Setuping World, MAVROS nodes, and four drones
```roslaunch drone_control fourDronesNodes.launch```

#### Running drone# node(single drone control)
```rosrun drone_control single_control (0-3)```

#### Running multi_control node(multi drone control)
```rosrun drone_control multi_control```

#### Running Google Tests(Optional)
```rosrun drone_control test_drone_control```

---

## Technical Breakdown
### Single Drone Control Structure
The file singleDrone.cpp stores the code for the ROS node that controls a single drone. Mutliple instances of this node allows for the independent control of multiple drones, with the aid of the multi-drone control node. There are two main sections of code in this ROS node, the state based control and the *interpretCommand*.

For the state based control, it must be known that a variable inside the node stores the state the drone is in. Below shows the different states the drone can be in, and depending on which one its in determines the drone's behavior. For example if node has the drone in the *HOVERING* state, then the drone would simply stay at, or very close, to a single point in a 3D space.

![image](https://user-images.githubusercontent.com/44033533/186802424-05f81d06-6408-4dae-ab81-4823004b6537.png)

As for transitioning the drone through all of the possiable states, a command needs to be sent to the node using the *dcontrol.msg* format, file found in the msg folder. A command inputted into the "command" section and a point, if necessary, into the target section. Not all commands need a target point inputted, such as *LIFT* or *STOP*. Below is a table of all of the possiable commands that could be sent and wheter they need a point or not. List of commands can also be found in the msg file.

### Commands

Command | Target
------- | ------
SHUTOFF | N/A
STOP | N/A
LIFT | Optional
LAND | N/A
CHECK | N/A
TRANSIT_ADD | Required
TRANSIT_NEW | Required

##### Explanation
- *SHUTOFF* command immediate disables the drone. **Useful for emergency shutoffs.** Note that drones can be shutoff high in the sky, causing them to fall quite a bit.
- *STOP* command stops a drone dead in its track by deleting all of its trajectories and putting it in the *HOVERING* state. **Useful for stopping a drone, but not disabling it**
- *LIFT* command lifts a drone off the ground into the air. If a point is given with a z > 0, then it will takeoff to that location. If no point is given it will take off 2m above its current position. 
- *LAND* command gently lands a drone onto the ground. Used generally at the end of flight or when a drone needs to be taken out of the sky safely.
- *CHECK* command will return the state the drone is currently in. It will use a *dresponse.msg* format, which will be talked about below.
- *TRANSIT_ADD* command will a trajectory for the drone to follow. A point is needed. If the drone is HOVERING and this command is called, it will calculate an optimal path from its current location to the inputted location using the [MAV Trajectory Generation](https://github.com/ethz-asl/mav_trajectory_generation) library. If the drone is currently in a trajectory, it will save that trajectory and go there once it is done with the current trajectory. Multiple trajectories can be added during or after a trajectory is complete.
- *TRANSIT_NEW* command is the same as the *TRANSIT_ADD* command, but with one difference. If its called while a drone is following a trajectory, it will immediately stop following that trajectory, stablize by trying to hover in place, delete all stored trajectories, and go to the new indented location. A point is needed. This is useful for following a new set of trajectories or moving toward a newly planned desitination.


Some states can automatically transition to another state. Such as, if the drone is in the *LIFTING_OFF* state and it has reached its intended point of takeoff, it will automatically go to the *HOVERING* state. Below are the automatic state transitions that happen when once a drone is put into a state in the *From* column.

##### Automatic State Transitions

From | To
---- | --
LIFTING_OFF | HOVERING
IN_TRANSIT* | HOVERING
LANDING | SHUTTING_DOWN

\*The *IN_TRANSIT* state will only go into the *HOVERING* state if there are no trajectories planned/stored after the current one.

![image](https://user-images.githubusercontent.com/44033533/186790602-2435b02e-b44d-4144-91de-e4c7d0182118.png)


### Package Structure

The ***drone_control*** package is structured into five main folders: helper, launch, msg, src and test folder. Below are quick summaries of each folder's purpose.
#### src
The src folder stores all the control code. Currently, the file singleDrone.cpp contains the ROS node code for each drone. It optimizes a path to a desired point, controls the state of the drone, and recieves/interprets control signals to the drone. The multiDrone.cpp file is used to control numerous drones by sending commands to each drone, and recieve any information from each drone. 
#### helper
The helper folder stores any mathematical computations or data conversions needed. This allows for testing and easy extensibility for future computations/conversions. Some of the functions written are the isStationary(...), isFlat(...), reachedLocation(...), and segmentToPoint(...).
#### test
The test folder contains tests for the helper functions in the helper folder. Some of the functions tested are mathematical computations, such as isStationary(...), while others are data conversions like in the segmentToPoint(...) function. GoogleTest is the test framework used.
#### launch
The launch folder contains the setup of the Gazebo simulation and MAVROS nodes. The fourDronesNodes.launch are example launch files used for testing the single/multi control structure.
#### msg
The msg folder contains the custom messages that are communicated between the individual drone nodes and the multi control node.

---
## Tools
  - C++
  - Catkin(Modified CMake)
  - Gazebo
  - Google Test
  - Git
  - Github
  - Linux
  - rqt(Debugging)
  - VSCode
  - VMWare

 ## Libraries/References
  - [Hanzhe Teng Wiki](https://wiki.hanzheteng.com/)
  - [MAV Trajectory Generation](https://github.com/ethz-asl/mav_trajectory_generation#bibliography)
  - [MAVLink: MAVROS](https://github.com/mavlink/mavros)
  - [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)
  - [ROS(Melodic)](http://wiki.ros.org/melodic)

---

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
