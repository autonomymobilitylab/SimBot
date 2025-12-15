# SimBot
Simbot is a ROS2 Humble simulation of the ABB Flexley Tug AMR, built in Ignition Gazebo Fortress. It replicates the robot’s real sensor setup, including multiple cameras and dual lidars, providing an accurate virtual model for development and testing.

# Requirements
<ul>
  <li>Ubuntu 22.04</li>
  <p></p>
  <li>ROS2 Humble</li>
  <p></p>
  <li>Ignition Gazebo Fortress</li>
  <p></p>
</ul>

# Installation 
For development of the package ensure your system is running Ubuntu 22.04, also ensure that ROS2 Humble and Ignition Gazebo Fortress is installed on your PC. If ROS2 Humble is not installed yet, please follow the instruction in the following link:
```bash
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
```
For installation of Gazebo Fortress, please follow the instruction in the following link:
```bash
https://gazebosim.org/docs/fortress/install_ubuntu/
```
After Installation of ROS2 and Ignition Gazebo, please also install the related packages and modules that are being used for this simulation as follows:
```bash
sudo apt-get install ros-humble-xacro
sudo apt-get install ros-humble-ros-gz-bridge
sudo apt-get install ros-humble-joint-state-publisher-gui
sudo apt-get install ros-humble-ros-gz-sim
sudo apt-get install ros-humble-teleop-twist-keyboard
```
After successfully installing all these packages in your PC,open a new terminal and source the environment:
```bash
source /opt/ros/humble/setup.bash
```
Then, open a terminal and create a workspace and navigate to that workspace.
```bash
mkdir -p ~/ ros2_ws
cd ros2_ws
```
Once inside the workspace, clone the package using the following command:
```bash
git clone https://github.com/autonomymobilitylab/SimBot.git
```
After cloning the package in your workspace, you'll see a directory named SimBot, navigate to that directory, and build the package using following commads. 
```bash
cd SimBot
colcon build --packages-select abb_flexley_description 
```
# Usage
After successfully building the package, you'll see build, install and log folder along with the already existing src folder. Now source your environment to install directory using following command.
```bash
source install/setup.bash 
```
Change the IP of the system to the local IP. 
```bash
export IGN_IP=127.0.0.1 
```
Now run the simulation using the following command. 
```bash
ros2 launch abb_flexley_description gazebo.launch.py 
```
This will launch the gazebo simulation.You'll see the gazebo and trolley inside the warehouse environment. The robot is now ready to be teleoperated around the environment, also all the sensors of the robot are now operational. For teleoperation of the robot, we need to run the control node. Open a new terminal, source the environment as described previously and then run the control node with the following command. 
```bash
ros2 run abb_flexley_description control_node
```
Once you have launched the control node, you'll see the commands to control the robot on terminal, the robot can now be teleoperated using the keyboard keys on the terminal. The sensors on the robot are now operational and you can see the output of the sensors by launching the publisher node in a new terminal, source the environment and run the node using this command. 
```bash
ros2 run abb_flexley_description sensor_publisher
```
You can attach the trolley with the robot, by moving the robot directly under the trolley and then using attach detach key control, this node won't work unless you are directly under the trolley. Once you are under the trolley, open a new terminal, source the environment and launch the node using following command. 
```bash
ros2 run abb_flexley_description attach_detach_key_control
```
This will launch the node, you'll see the commands for attachment and detachment in the terminal, if you are close to the trolley, you can attach it using the keyboard keys and teleoperate the robot with the trolley inside warehouse. The robot can then be detached with the keyboard keys as well.  You can also visualize the output of the sensors in RViz. Before launching RViz, we need to launch two nodes to unify the output of the lidars and the cameras. For lidars, we need to launch the lidar frame merger node. Open a new terminal, source the environment, and then launch the node with the following command. 
```bash
ros2 run abb_flexley_description lidar_frame_merger
```
This wil merge the output of the two lidars and will publish them to a single topic which we can visualize in RViz. For cmaeras, we need to launch another node, open a new terminal, source the environment, then launch the node with the following command. 
```bash
ros2 run abb_flexley_description multi_camera_stitcher
```
Once you have launched both of these nodes, now you can run the RViz to visualize the output of the sensors with the following command. 
```bash
ros2 launch abb_flexley_description display.launch.py
```
You can add the camera and the lidar in the RViz and select the corresponding topic to visualize the output. While running the nodes, if you encounter the error stating "no executable found", open a new terminal, navigate to the directory containing the nodes using 'cd' with the folder path and once inside that directory,then run the following command to make the node executable.
```bash
chmod +x node_name.py
```
This will make the node executable and you can then rerun the node in the terminal. 
# Demo
The robot's simulation can be seen in the video below, demonstrating the robot's teleoperation in the warehouse environment.
<br><br>
![Screencastfrom2025-08-11162700-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/e7a10525-02b9-431d-9332-7f5402593aec)
<br><br>
The attachement and detachment of the trolley with the robot can be seen below.
<br><br>
![Robotwithtrolley-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/0e8dcb6e-887a-4f64-a86f-055e7ce23619)
<br><br>
The output of the robot's sensor in RViz is shown in the video below.
<br><br>
![ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/9c9630cd-c701-4642-ac9b-486dc6afb209)
<br><br>


