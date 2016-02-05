# msl_gazebo_simulator
The initial code base is copied from NuBots MSL Repository: https://github.com/weijia-yao/single_nubot_gazebo
The corresponding paper can be found here: https://nubot.trustie.net/attachments/download/84264/A%20Simulation%20System%20Based%20on%20ROS%20and%20Gazebo%20for%20RoboCup%20Middle%20Size%20League.pdf

In order to make this package run under Ubuntu 14.04 LTS with ROS Indigo you have to execute the following commands (assuming that you have ros-indigo-desktop-full installed):

    sudo apt-get remove gazebo2*
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get install ros-indigo-gazebo5-ros-pkgs ros-indigo-gazebo5-ros-control
