
**This file describes work done and steps to perform test on the laser_filters package for ROS2 (Crystal).**


**Overview:**

  laser_filters is a package of assorted filters designed to operate on 2D planar laser scanners, which use the sensor_msgs/LaserScan type.


**List of changes from ROS1 to ROS2:**

  The changes has been done by following the Migration guide for ROS2( https://index.ros.org/doc/ros2/Migration-Guide/ ).

  1. Migrated all header and source files into ROS2 style.
  2. Migrated .yaml files as per ROS2 style.
  3. Migrated .launch files into launch.py for ROS2.
  4. Migrated CMakeLists.txt & package.xml for ROS2 changes.
  5. Migrated C++ gtests for ROS2 Changes.


**Pre-requisites:**

  1. System should have installed Crystal distro. Check out installation instructions and tutorials
      https://index.ros.org/doc/ros2/.
  2. System should have checked-out & built ros2 branches for laser_filters, filters, laser_geometry pkg
  (Refer Steps below).

    Steps to checkout laser_geometry package:
      1. Open the new terminal and run below commands.
      2. mkdir -p laser_geometry_ws/src
      3. cd ~/laser_geometry_ws/src
      4. git clone git@github.com:vandanamandlik/laser_geometry.git -b ros2-devel
	
    Steps to checkout filters package:
      1. Open the new terminal and run below commands.
      2. mkdir -p filters_ws/src
      3. cd ~/filters_ws/src
      4. git clone git@github.com:swatifulzele/filters.git -b ros2_devel

    Steps to checkout laser_filters package:
      1. Open the new terminal and run below commands.
      2. mkdir -p laser_filters_ws/src
      3. cd ~/laser_filters_ws/src
      4. git clone git@github.com:Rohita83/laser_filters.git -b ros2

    Steps to Build filters & laser_geometry package:
      1. cd ~/filters_ws/src/filters
      2. source /opt/ros/crystal/setup.bash
      3. colcon build
      4. cd ~/laser_geometry_ws/src/laser_geometry
      5. colcon build

    Steps to Build laser_filters package:
      1. cd ~/laser_filters_ws/src/laser_filters
      2. source /opt/ros/crystal/setup.bash
      3. source ~/laser_geometry_ws/src/laser_geometry/install/setup.bash
      4. source ~/filters_ws/src/filters/install/setup.bash
      5. mv /opt/ros/crystal/include/laser_geometry/ /opt/ros/crystal/include/laser_geometry_bkp/
      5. colcon build


**Design changes:**

  1. The parameters :
    Parameters for laser_filters are taken from yaml file only. So laser_filter type is configured using node paramter and corresponding code and parameter acquiring changes have been added to devired class of laser_filters.

  2. The yaml file is redesigned so as to configure multiple laser_filters in sequence.
      Refer below extract of as a sample yaml file:
	
	intensity_filter_chain:
	  ros__parameters:
            name: intensity_threshold
            type: laser_filters/LaserScanIntensityFilter
            params: 
              lower_threshold: 0.5
              upper_threshold: 3.0
              disp_histogram: 1

        interp_filter_chain:
          ros__parameters:
            name: interpolation
            type: laser_filters/InterpolationFilter


  3. Xmlrpc variables replaced by string types variables and required modification/changes incorporated.


**Future Work:-**

  1. Migrate laser_filters python tests to ros2.


**Limitations:**

  1. Include path of laser_geometry package from Ros2 installation directory needs to be temporarily moved.
    This is required as laser_geometry is already available in Ros2 Crystal installation.
    And cmake <pkg>_INCLUDE_DIRS some how just finds the includes from default Ros2 install path only.
    This prevents laser_filters package from finding the required header files of latest checked-out laser_geometry package in pre-requisite step.
  2. colcon test does not work as launch.py files can not be executed/added with CMakeLists.txt as of now.
  3. Each test/launch.py files have been tested independently.


**TESTING:-**

  Here launch files are used independently. Following are the steps to run the test cases independently:
  1. Set the path:
  
    $ cd ~/laser_filters_ws/src/laser_filters
    $ source ./install/setup.bash 

  2. To run Test cases related to the laser_filters use following command:
    
    $ ros2 launch laser_filters test_scan_filter_chain.launch.py


