^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package laser_filters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.8 (2024-11-12)
------------------
* Merge pull request `#202 <https://github.com/ros-perception/laser_filters/issues/202>`_ from Oscar-Robotics/ros2
  Using NaN instead of range_max+1 to remove scans in angular_bounds_filter_in_place
* Merge branch 'ros2' of https://github.com/Oscar-Robotics/laser_filters into ros2
* added comment
* adding replace_with_nan param
* Merge pull request `#203 <https://github.com/ros-perception/laser_filters/issues/203>`_ from jonbinney/bugfix/add-launch-testing-depend
  Add missing dependency on ament launch testing
* Add missing dependency on ament launch testing
* Merge pull request `#189 <https://github.com/ros-perception/laser_filters/issues/189>`_ from eurogroep/feat/ros2-port-all-filters
  Feature: ros2 port for all filters
* Merge pull request `#201 <https://github.com/ros-perception/laser_filters/issues/201>`_ from YBachmann/spatial_median_filter
  Spatial median filter
* Added example launchfiles (xml+py) and example parameter yaml file. Added comment and warning when ensuring window_size\_ is odd
* Merge pull request `#7 <https://github.com/ros-perception/laser_filters/issues/7>`_ from jonbinney/fix-port-all-filters-test
  Fix race condition in tests
* using NaN instead of range_max+1 to remove scans
* Fix race condition in tests
  We need to publish scans repeatedly in case the filter chain runs and
  processes the output scan before we finish subscribing to that topic.
* Durability policy for publisher in speckle filter test
* Fix typo
* Expand description of filter in laser_filters_plugins.xml
* Don´t declare+initialize window_size before getParam call
* Use nan and infinity values for the median if they make up the majority of the window
* Merge pull request `#162 <https://github.com/ros-perception/laser_filters/issues/162>`_ from wolfv/do_not_use_not
  Do not use "not" as it is not defined in MSVC
* Removed unused file
* Removed node interface
* Fixed test
* Merge branch 'spatial_median_filter' of https://github.com/YBachmann/laser_filters into spatial_median_filter
* Revert "Initial version for distance moving window filter."
  This reverts commit 16118ea4a86e6794a9b3f7ec118c1dc2140e1307.
* Merge branch 'ros-perception:ros2' into spatial_median_filter
* Added LaserScanMedianFilter
* Merge pull request `#195 <https://github.com/ros-perception/laser_filters/issues/195>`_ from bjsowa/lazy-subscription
  Add Lazy subscription
* Keep the old publisher QoS settings
* Initial version for distance moving window filter.
* Check rclcpp version instead of ROS distribution
  Co-authored-by: Jonathan Binney <jon.binney@gmail.com>
* Add lazy subscription to filter chains
* Merge pull request `#199 <https://github.com/ros-perception/laser_filters/issues/199>`_ from bjsowa/sort-up-ros-parameters
  Sort up ROS parameters for filter chains
* Merge pull request `#198 <https://github.com/ros-perception/laser_filters/issues/198>`_ from bjsowa/use-templated-node-executables
  Create node executables using rclcpp_component macros
* Merge remote-tracking branch 'origin' into sort-up-ros-parameters
* Add a comment about rclcpp components macro
* Merge pull request `#197 <https://github.com/ros-perception/laser_filters/issues/197>`_ from bjsowa/shared-from-this-fix
  Don't use shared_from_this in the constructor
* Merge remote-tracking branch 'upstream/ros2' into feat/ros2-port-all-filters
* Create node executables using rclcpp_component macros
* Don't use shared_from_this in the constructor
* Sort up ROS parameters for filter chains
* Merge pull request `#196 <https://github.com/ros-perception/laser_filters/issues/196>`_ from jonbinney/update-ros2-test-distros
  Update list of ros2 distros to test on
* Update list of ros2 distros to test on
* Merge pull request `#188 <https://github.com/ros-perception/laser_filters/issues/188>`_ from jcarlosgm30/feature/component-support
  feature: components support
* Copy getPointCloud2FieldIndex function
* Added dependency
* Changed RCLCPP_WARN_THROTTLE
  Co-authored-by: Błażej Sowa <bsowa123@gmail.com>
* Update include/laser_filters/polygon_filter.h
  Co-authored-by: Błażej Sowa <bsowa123@gmail.com>
* feat: components support
* file reorganization: filter chains as libraries
* Port scan shadows filter unit tests to ROS2
* Port speckle filter unit tests to ROS2
* Port speckle filter launch tests to ROS2
* Removed commented print statements
* Port sector filter to ROS2
* Port scan blob filter to ROS2
* Port polygon filter to ROS2
* Merge pull request `#186 <https://github.com/ros-perception/laser_filters/issues/186>`_ from JosefGst/time_stamp
  Fix time stamp issue for angular filter
* change to rclcpp Time
* add endline
* do time math with duration
* add angular filter example
* fix time stamp increment
* Do not use "not" as it is not defined in MSVC
* Contributors: Berend Kupers, Błażej Sowa, Dr. Denis Štogl, Jon Binney, Jonathan Binney, JosefGst, MAHA Maia, Wolf Vollprecht, Yannic Bachmann, Yohan Belanger, berend-kupers, josegarcia

2.0.7 (2023-07-31)
------------------
* Escape invalid xml in laser_filters_plugins.xml
* Contributors: Calder Phillips-Grafflin

2.0.6 (2023-03-18)
------------------
* Added declaration of parameters
* Reduce computation cost of ScanShadowsFilter
* Update scan_to_cloud_filter_chain.cpp
  As of Eloquent a timer interface is required for the tf buffer.
  https://docs.ros.org/en/galactic/Releases/Release-Eloquent-Elusor.html#tf2-buffer
* Contributors: Atsushi Watanabe, Jon Binney, Jonathan Binney, Riccardo Tornese, brandonbeggs

2.0.5 (2022-05-26)
------------------
* Remove remaining uses of boost.
  All of that functionality is now available in std:: .  Also, this
  should fix the build on RHEL.
* Contributors: Chris Lalancette

2.0.4 (2022-04-08)
------------------
* Add a sensor_msgs dependency to test_scan_filter_chain
* adding support for invert-parameter to select if points within or outside of box are kept
* Contributors: Chris Lalancette, Jonathan Binney, Nikolas Engelhard

2.0.3 (2021-10-19)
------------------
* Add top level license file
  The license is the same as it always has been; this commmit just copies
  the license text from the source files into a top level LICENSE file to
  make it clear.
* Contributors: Jon Binney

2.0.1 (2021-10-18)
------------------
* Add build depend on ament_cmake_auto
* Contributors: Jon Binney

2.0.0 (2021-10-13)
------------------
* Enable CI for foxy, galactic and rolling distros
* Remove unneeded find_package of pcl_conversions
* Port speckle filter to ros2
* Remove pointcloud footprint filter
  It has been deprecated for years, and is the only filter that depends on
  pcl_ros. Removing it means we don't have to install the 500MB of
  dependencies that pcl brings in on CI.
* ROS2 migration (foxy)
* Make laser_filters build for ros2 (on windows 10)
* Updated deprecated pluginlib macros to avoid warning messages
* Contributors: Brian Fjeldstad, Jon Binney, Jonathan Binney, Nick Lamprianidis, Nicolas Limpert, Patrick Lascombe, Rein Appeldoorn, hang

1.8.5 (2017-09-06)
------------------
* rename parameter to be more descriptive
* change range_filter to infinity for it to work with obstacle_layer
  if you use the ´inf_is_valid´ parameter raytracing is still possible for
  scans out of the window.
  Usefull for laserscanners that may deliver ranges > range_max ... or
* Fix a small typo in one of the test cases.
* Add LaserScanMaskFilter.
  This commit adds LaserScanMaskFilter that removes points on directions defined in a mask, defined as a parameter, from a laser scan.
  It can be used to remove unreliable points caused by hardware related problems for example scratches on an optical window of the sensor.
* Contributors: Atsushi Watanabe, Hunter L. Allen, Jannik Abbenseth, Jonathan Binney

1.8.4 (2017-04-07)
------------------
* Specify packages names for filters in tests
* Use std:: namespace for c++11 compat.
* Contributors: Jon Binney, Jonathan Binney, Mike Purvis

1.8.3 (2016-05-20)
------------------
* Replaced the invalid value of scans for the footprint_filter by NaN
* Contributors: Alain Minier

1.8.2 (2016-04-06)
------------------
* Remove unneeded eigen and cmake_modules
  Nothing was actually compiling against eigen.
* Contributors: Jonathan Binney

1.8.1 (2016-03-26)
-----------
* Remove deprecated warning from footprint filter
* catkin_make requires cmake_modules in run_depends
* Restore cmake_modules build dependency
* Update package.xml
* Update maintainer email address
* Add Travis CI config
* Update scan_to_scan_filter_chain.cpp
* only publish result if filter succeeded
* Contributors: Isaac I.Y. Saito, Jon Binney, Jonathan Binney, Kei Okada, Naveed Usmani, asimay

1.7.4 (2015-12-17)
------------------
* [intensity_filter.h] fix: check if cur_bucket value is out of index of histogram array
* [intensity_filter.h] refactor codes; clearify by using boolean to enable/disable displaying histogram
* scan_to_scan_filter_chain: make tf filter tolerance customizable
  0.03 is completely arbitrary and too small in my case.
* scan2scan filter: only publish result if filter succeeded
* added cartesian box filter
* add check inf or nan of input laser_scan intensities
* scan_to_scan_filter_chain: Only subscribe to /tf if requested by parameter
* Contributors: Furushchev, Jonathan Binney, Kevin Hallenbeck, Sebastian Pütz, Vincent Rabaud, Yuto Inagaki, v4hn

1.7.3 (2014-09-06)
------------------
* Added new filter LaserScanAngularRemovalFilterInPlace to remove sections of a LaserScan
* Contributors: Kevin Hallenbeck, Vincent Rabaud

1.7.2 (2014-06-24)
------------------
* Merge pull request `#19 <https://github.com/ros-perception/laser_filters/issues/19>`_ from v4hn/no-DEPENDS-dependency
  remove superfluous DEPENDS
* remove superfluous DEPENDS
  There never was a DEPENDS flag in add_dependencies...
* Contributors: Vincent Rabaud, v4hn

1.7.1 (2014-06-06)
------------------
* Tests expect NaN for invalid ranges
* Modify intensity, scan shadow, and range filters to set invalid values to NaN
* Contributors: Allison Tse, Jonathan Binney

1.6.14 (2014-03-04)
-------------------
* fix compilation on some platforms
* Contributors: Vincent Rabaud

1.6.13 (2014-03-02)
-------------------
* separate tests
* remove PCL dependency
* Don't check the intensities
  The intensities are not used in the range filter.
  Furthermore, some laser don't have intensities ---e.g hokuyo URG-04LX-UG01---, so this fails for them.
* Contributors: Enrique Fernández Perdomo, Vincent Rabaud

1.6.12 (2013-12-24)
-------------------
* "1.6.12"
* Merge pull request `#13 <https://github.com/ros-perception/laser_filters/issues/13>`_ from v4hn/less_startup_noise
  footprint_filter: print less tf warnings
* footprint_filter: print less tf warnings
  On startup this filter produces about two pages of console output
  (ROS_ERRORs) on ExtrapolationExceptions because the listener is
  not setup yet. This commit reduces this to throttled info messages
  until the transform works for the first time.
* compile rostests with add_executable, not catkin_add_gtest
* Contributors: Jon Binney, Vincent Rabaud, v4hn

1.6.11 (2013-07-19)
-------------------
* Merge pull request `#12 <https://github.com/ros-perception/laser_filters/issues/12>`_ from ros-perception/fix_angles_dep
  missing dependency break isolated build without --install
* add missing dependency on the angles package
* Contributors: William Woodall

1.6.10 (2013-06-27 16:11)
-------------------------
* install scan_to_scan filter
* Contributors: Jon Binney

1.6.9 (2013-06-27 09:36)
------------------------
* Merge pull request `#11 <https://github.com/ros-perception/laser_filters/issues/11>`_ from piyushk/patch-1
  Fixed typo in exported library names
* Fixed typo in exported library names
  It's a pretty minor error, but unfortunately breaks the system release due to nonexistent lib_point_cloud_filters.so
* Contributors: Piyush Khandelwal, Vincent Rabaud

1.6.8 (2013-05-30)
------------------
* Merge pull request `#7 <https://github.com/ros-perception/laser_filters/issues/7>`_ from ros-perception/scan-scan-filter-chain
  Restored scan_to_scan_filter_chain executable lost in the catkinization.
* Restored scan_to_scan_filter_chain executable lost in the catkinization.
* Contributors: Dave Hershberger, jonbinney

1.6.7 (2013-05-24)
------------------
* bump version for bugfix
* Merge pull request `#6 <https://github.com/ros-perception/laser_filters/issues/6>`_ from jonbinney/install_include
  added install rule for headers in cmakelists
* added install rule for headers in cmakelists
* Contributors: Jon Binney, jonbinney

1.6.6 (2013-05-23)
------------------
* bumped version for hydro release
* Merge pull request `#5 <https://github.com/ros-perception/laser_filters/issues/5>`_ from jonbinney/build_fixes
  fixed rostests
* fixed rostests
* Merge pull request `#4 <https://github.com/ros-perception/laser_filters/issues/4>`_ from jonbinney/catkinized
  Catkinized
* fixes to cmakelists
* deleted unneeded cmake file
* catkinized laser_filters
* Contributors: Jon Binney, jonbinney

1.5.7 (2013-07-11 15:22)
------------------------
* restore dependecy on laser_geometry
* Contributors: Jon Binney

1.5.6 (2013-07-11 15:06)
------------------------
* fix crash with negative values
* Merge pull request `#3 <https://github.com/ros-perception/laser_filters/issues/3>`_ from YoheiKakiuchi/groovy-devel
  add range_filter to laser_scan_filters.cpp
* comment out laser_geometry (it was needed to use this package in fuerte)
* add range_filter to laser_scan_filters.cpp
* Contributors: Vincent Rabaud, YoheiKakiuchi

1.5.5 (2012-10-12 11:16)
------------------------
* releasing 1.5.5
* added missing dependency on laser_geometry
* Contributors: Dave Hershberger

1.5.4 (2012-10-12 10:38)
------------------------
* added .gitignore
* created stack.xml and added stuff for unary-stack-ification
* revert to the angles package
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@40134 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* fix the non-inclusion of PCL
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@40128 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* more angles fixing
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@40123 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Changing the name of the incident angle correction parameter to make some amount of sense
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@38975 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Fixing the scan to cloud filter chain to actually work properly with PointCloud2 messages
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@38974 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* added param for hack
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@38655 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* add missing boost links, needed for catkin, but backward compatible
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@38615 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* - first try at converting the PointCloud to PointCloud2
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@38479 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* use the new bullet and eigen conventions
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@38342 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Removing deprecation warnings
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@35256 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Killing deprecated preservative param
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@35241 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Added Ubuntu platform tags to manifest
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@29657 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Removing deprecated usage of ~ for `#3771 <https://github.com/ros-perception/laser_filters/issues/3771>`_
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@27729 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* adding test for array filter
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@26944 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* adding shadow filter test
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@26874 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* adding test for interp filter
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@26872 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* adding simple tests
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@26866 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* adding tests but checking in with CMake comeented out for now
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@26803 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Updating stack/manifest.xml files
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@26801 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Adding a angular bounds filter that allows scans to be truncated to be within a user-specified range.
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@26736 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Added link against boost::system, to fix build on OS X
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@25628 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Removing old/unused/broken code from scan_to_cloud_filter_chain.
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24700 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Checking in the node diagrams.
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24687 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Making scan_to_cloud_filter_chain robust to a likely user migration error.
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24660 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Making the scan_to_scan_filter_chain use scan_filter_chain.
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24659 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Making scan_to_cloud_filter_chain adhere to new API from http://www.ros.org/wiki/laser_filters/Reviews/2009-9-28_API_Review
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24629 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Making laser_filters adhere to new API from http://www.ros.org/wiki/laser_filters/Reviews/2009-9-28_API_Review
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24627 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* A little more laser_filter code cleanup.
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24485 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Cleaning up generic_laser_filter_node code since it is used as part of the laser_filters tutorial.
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24482 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Leaving point_cloud_footprint_filter_example in laser_pipeline as well for now.
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24415 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Adding back int the footprint_filter_examples despite deprecation to avoid breaking people using deprecated plugins.
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24389 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Fixing laser_filter to use tf::MessageFilter instead of tf::MessageNotifier and deprecating the footprint filters.
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24388 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Removing invalid linking from laser_filters.
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24353 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Deprecating preservative parameter.
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24324 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* capitalization in filter description
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24312 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Convert to NodeHandle
  git-svn-id: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk@24160 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Adding a filter to interpolate between laser readings to generate range readings for scans that return errors
  git-svn-id: https://code.ros.org/svn/ros-pkg/pkg/trunk/stacks/laser_pipeline@23875 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Merging in remaining missing contents for laser_piple that svn ignored on the first merge.
  git-svn-id: https://code.ros.org/svn/ros-pkg/pkg/trunk/stacks/laser_pipeline@23510 eb33c2ac-9c88-4c90-87e0-44a10359b0c3
* Contributors: Brian Gerkey, Dave Hershberger, Eitan Marder-Eppstein, Eric Berger, Jeremy Leibs, Josh Faust, Kaijen Hsaio, Melonee Wise, Vincent Rabaud
