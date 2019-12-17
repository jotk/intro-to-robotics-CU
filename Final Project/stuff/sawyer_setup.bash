sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt-get update

sudo apt-get --assume-yes install ros-melodic-desktop-full git gazebo9 ros-melodic-gazebo-ros-control ros-melodic-gazebo-ros-pkgs ros-melodic-ros-control ros-melodic-control-toolbox ros-melodic-realtime-tools ros-melodic-ros-controllers ros-melodic-xacro python-wstool ros-melodic-tf-conversions ros-melodic-kdl-parser python-argparse python-vcstools python-rosdep ros-melodic-control-msgs ros-melodic-joystick-drivers ros-melodic-tf2-ros ros-melodic-rviz ros-melodic-cv-bridge ros-melodic-actionlib ros-melodic-actionlib-msgs ros-melodic-dynamic-reconfigure ros-melodic-trajectory-msgs ros-melodic-rospy-message-converter ros-melodic-effort-controllers python-rosinstall python-rosinstall-generator  build-essential


# make workspace dir if it is not already there

mkdir -p ~/ros_ws/src

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "export ROS_WORKSPACE=/home/`whoami`/ros_ws" >> ~/.bashrc
source ~/.bashrc
#Download intera SDK on workstation (ros_ws is your ros workspace)

cd ~/ros_ws/src
sudo rosdep init
rosdep update

wstool init .
wget https://raw.githubusercontent.com/cairo-robotics/sawyer_robot/master/sawyer_robot.rosinstall
wstool merge sawyer_robot.rosinstall
wstool update

#Source and build
cd ~/ros_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
catkin_make

#Copy the intera.sh script
cp ~/ros_ws/src/intera_sdk/intera.sh ~/ros_ws

