# msl_gazebo_simulator
clone from nubots simulator

In order to make this package run under Ubuntu 14.04 LTS with ROS Indigo you have to execute the following commands (assuming that you have ros-indigo-desktop-full installed):

    sudo apt-get remove gazebo2*
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get install ros-indigo-gazebo5-ros-pkgs ros-indigo-gazebo5-ros-control
