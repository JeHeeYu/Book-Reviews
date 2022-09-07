# Robots-with-ROS
　Robot with ROS 교재 관련 정리 Repository
<br>
## Environment
　Hardware :Jetson Xavier AGX
<br>
　OS : Ubuntu 18.04
<br>
　ROS : Melodic
<br>
## Installation
<pre>
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo apt update
sudo apt install ros-melodic-desktop-full
</pre>
## Environment Setting
<pre>
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo rosdep init
rosdep update
</pre>
## Turtlebot3 and Gazebo
<pre>
sudo apt-get install ros-melodic-turtlebot3-gazebo
sudo apt-get install ros-melodic-turtlebot3-msgs
sudo apt-get install ros-melodic-turtlebot3
</pre>
## Workspace
<pre>
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

cd ~/catkin_ws
catkin_make
</pre>
　출처 : Robot Wiwith ROS(jpub)
