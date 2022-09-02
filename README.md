# UCR Drone Control

This repository combines a trajectory planning, communication protocol, and robotics libraries for a simple interface to control multiple drones. Our code is built with the [ROS (Melodic)](http://wiki.ros.org/melodic) and [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) frameworks to develop the drone control software. The [MAV Trajectory Generation](https://github.com/ethz-asl/mav_trajectory_generation) library generates an optimized, minimum snap, path for our drones to follow. Finally, [MAVLink (MAVROS)](http://wiki.ros.org/mavros) is our primary messaging protocol, interfacing with the drones.

![droneDemo1](https://user-images.githubusercontent.com/44033533/188036430-74ee8e96-2bcc-49c3-9673-c8df9dbdc5bf.gif)

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

#### Setup World, MAVROS nodes, and four drones
```roslaunch drone_control fourDronesNodes.launch```

#### Running drone# node(single drone control)
```rosrun drone_control single_control (0-3)```

#### Running multi_control node(multi drone control)
```rosrun drone_control multi_control```

#### Running Google Tests(Optional)
```rosrun drone_control test_drone_control```

---

## Technical Breakdown

This breakdown will help explain essential information needed to interface, commands and response, with the single drone control node.

### Multi Drone Control Structure
The multiDrone.cpp file stores the code for the ROS node that controls all of the single drone control nodes. In the current file, an example of controlling four nodes is given. The file uses ***keyboard inputs*** in order to control the drone. Responses are used to see when the drone has shutoff. Responses will be talked about later in the technical breakdown.

### Single Drone Control Structure
The singleDrone.cpp file stores the code for the ROS node that controls a single drone. Multiple instances of this node allow for the independent control of multiple drones, with the aid of the multi-drone control node. Below is a visaulization of the multi_control_node and single_drone_control(drone0) node interactions through certain topics. 

![image](https://user-images.githubusercontent.com/44033533/186790602-2435b02e-b44d-4144-91de-e4c7d0182118.png)

The single drone control node is centered around a state-based control approach. For the state-based control, it must be known that a variable inside the node stores the state the drone is in. Below shows the different states the drone can be in, and depending which one the drone is in determines the drone's behavior. For example, if the node has the drone in the *HOVERING* state, then the drone would simply stay at, or very close, to a single point in the 3D space. The *GROUND_IDLE* state is the state the drone is first initialized to on the ground. All other states should be self-explanatory. 

![image](https://user-images.githubusercontent.com/44033533/187813657-ca4ffe3f-d300-496e-ad13-0686a2ff9a6f.png)

As for transitioning the drone through the possible states, a command needs to be sent to the node using the *dcontrol.msg* format, which will then be interpreted by the *interpretCommand* function. A command, string, is inputted into the "command" section, and a point, three floats, into the target section. Not all commands need a target point inputted, such as *LIFT* and *STOP*. Below is a table of all of the possible commands that could be sent and whether they need a point to be sent alongside it. A list of commands can also be found in the msg file. 

### Commands

Command | Target
------- | ------
SHUTOFF | N/A
STOP | N/A
LIFT | Optional
LAND | N/A
TRANSIT_ADD | Required
TRANSIT_NEW | Required

##### Explanation
- *SHUTOFF* command immediately disables the drone. **Useful for emergency shutoffs.** Note that drones can be shut off high in the sky, causing them to fall quite a bit.
- *STOP* command stops a drone dead in its track by deleting all of its trajectories and putting it in the *HOVERING* state. **Useful for stopping a drone, but not disabling it.**
- *LIFT* command lifts a drone off the ground into the air. If a point is given with a z > 0, then it will take off to that location. If no point is given it will take off 2m above its current position. A point is not needed to liftoff properly.
- *LAND* command gently lands a drone onto the ground. Used generally at the end of a flight or when a drone needs to be taken out of the sky safely.
- *TRANSIT_ADD* command will generate a trajectory for the drone to follow, to a given target. A point is needed. If the drone is HOVERING and this command is called, it will calculate an optimal path from its current location to the target location using the [MAV Trajectory Generation](https://github.com/ethz-asl/mav_trajectory_generation) library. If the drone is currently in a trajectory, it will save that trajectory and go there once it is done with the current trajectory. Multiple trajectories can be added during or after a trajectory is complete.
- *TRANSIT_NEW* command is the same as the *TRANSIT_ADD* command, but with one difference. If it is called while a drone is following a trajectory, it will immediately stop following that trajectory, stabilize by hovering in place, delete all stored trajectories, and go to the new indented location. A point is needed. This is useful for following a new set of trajectories or moving toward a newly planned course.

### Command Behaviors

Although the commands have certain outcomes, as explained above, they only work in certain situations. For example, a *TRANSIT_NEW* or *TRANSIT_ADD* command will not execute if the drone has not lifted off. For each command, there are prerequisite states that the drone must be in. Below are commands with the states that will work. Note the *SHUTTING_DOWN* state is not listed since the drone will turn off once in that state.

##### Commands and Prerequisite States
- SHUTOFF: GROUND_IDLE, LIFTING_OFF, HOVERING, IN_TRANSIT, LANDING
- STOP: LIFTING_OFF, HOVERING, IN_TRANSIT, LANDING
- LIFT: GROUND_IDLE
- LAND: LIFTING_OFF, HOVERING, IN_TRANSIT
- TRANSIT_ADD: HOVERING, IN_TRANSIT
- TRANSIT_NEW: HOVERING, IN_TRANSIT

### Automatic State Transitions

Some states can automatically transition to another state. Such as, if the drone is in the *LIFTING_OFF* state and it has reached its intended point of takeoff, it will automatically go to the *HOVERING* state. Below are the automatic state transitions that happen once a drone is put into a state in the *From* column.

From | To
---- | --
LIFTING_OFF | HOVERING
IN_TRANSIT* | HOVERING
LANDING | SHUTTING_DOWN

\*The *IN_TRANSIT* state will only go into the *HOVERING* state if there are no trajectories planned/stored after the current one. If there is multiple trajectories planned, it will stay in the *IN_TRANSIT* state to all planned trajectories.

### Responses
Responses are a way to send information from a single drone control node to the multi drone control node. Responses communicate the drone's state, position, and target. This may be useful for verifying if a message was received, a command was executed, or a position was reached. Below is the format of such messages.

<ins>**Format** (Data Type & Name)</ins>

uint8 *state*<br />
geometry_msgs/Pose *pose*<br />
geometry_msgs/Pose *target*<br />

- *state* corresponds to the state the drone is in, see image above to see which number corresponds to which state.
- *pose* is the current position(x,y,z) of the drone, relative to its origin.
- *target* is the position the drone is trying to go to, usually the end of the.

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
