# vl530lx-ROS-driver

### Build from Source

1. [Install ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) and the following build tools.

        sudo apt-get install python-wstool python-catkin-tools 
	
1. Re-use or create a catkin workspace:

        export CATKIN_WS=~/ws_catkin
        mkdir -p $CATKIN_WS/src
        cd $CATKIN_WS/src

1. Download the repository and the submodules:

        git clone --recursive git@github.com:rapyuta-robotics/vl53l0x_ros_driver.git
        wstool init .
        rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO

1. Clone and compile libsoc:

        git clone https://github.com/jackmitch/libsoc.git libsoc.git
        cd libsoc.git
        autoreconf -i      
        ./configure
        make
        make install

1. Copy the rules for proximity sensor
	sudo cp 60-i2c-tiny-proximity.rules /lib/udev/rules.d/

1. Configure and build the workspace:

        cd $CATKIN_WS
        catkin config --extend /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=Release
        catkin build 
 

1. Source the workspace.

        source $CATKIN_WS/devel/setup.bash

### Read data from the sensors
1. Run the launch:

        roslaunch vl53l0x_driver vl53l0x_driver.launch 

1. Read the ROS message of sensor 1:

		➜  ~ rostopic echo /sensor_data_1
		header: 
		  seq: 11
		  stamp: 
		    secs: 1563942747
		    nsecs: 325853385
		  frame_id: "sensor1"
		proximity: 8.19099998474
		field_of_view: 0.436331987381
		min_range: 0.0299999993294
		max_range: 2.0
		---

	
	



