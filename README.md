# vl530lx-ROS-driver

## Install directly

### Build from Source

1. [Install ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) and the following build tools.

        sudo apt-get install python-wstool python-catkin-tools 
        
1. Re-use or create a catkin workspace:

        export CATKIN_WS=~/ws_catkin
        mkdir -p $CATKIN_WS/src
        cd $CATKIN_WS/src

1. Download the repository:

        git clone git@github.com:rapyuta-robotics/vl53l0x_ros_driver.git
        wstool init .
        rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO
        
       

1. Configure and build the workspace:

        cd ..
        catkin config --extend /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=Release
        catkin build 
 

1. Source the workspace.

        source ./devel/setup.bash

### Read data from the sensors

1. 

                roslaunch vl53l0x_driver vl53l0x_driver.launch

