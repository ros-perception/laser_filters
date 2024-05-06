^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package laser_filters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
2.1.1 (2024-05-06)
------------------
* fix(polygon_filter): Fixed footprint subscriber for static polygon filter
* Contributors: Berend Kupers

2.1.0 (2024-05-02)
------------------
* Added reconfigure callbacks for all filters
* Changed filter base interface
* Added launch tests
* Contributors: Berend Kupers

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
* Contributors: Brian Fjeldstad, Jon Binney, Jonathan Binney, Nick Lamprianidis, Nicolas Limpert, Patrick Lascombe, Rein Appeldoorn

1.11.0 (2023-12-19)
-------------------
* feat(footprint-listener): Add subscriber for footprint topic (`#2 <https://github.com/eurogroep/laser_filters/pull/2>`_)

1.10.0 (2023-06-20)
-------------------
* feat(filters): Debug log filter duration (`#1 <https://github.com/eurogroep/laser_filters/issues/1>`_)
* Merge pull request `#177 <https://github.com/eurogroep/laser_filters/issues/177>`_ from lucasw/ubuntu2204
  use class_list_macros.hpp instead of .h
* use class_list_macros.hpp instead of .h
* Merge pull request `#171 <https://github.com/eurogroep/laser_filters/issues/171>`_ from ros-o/obese-devel
  [ROS-O] various cleanup
* Merge pull request `#169 <https://github.com/eurogroep/laser_filters/issues/169>`_ from marip8/fix/nodelet-remapping
  ROS1 Nodelet Updates
* add virtual destructor to avoid warning
  as these are pure method classes it's actually not an issue,
  but still a valid bug for clang to complain about.
* update header include paths for pluginlib
  the non-hpp headers have been deprecated since kinetic.
* remove global usage of boost's _1
  since std::bind was around boost's _1 has been deprecated as global symbol.
  Debian removed implicit support for it in their ROS packages and fails without
  this patch. Sadly message_filters does not (yet?) work with generic lambdas,
  so I added the namespaces for these cases.
* build rostest with canonical instructions
  to avoid linker errors on Debian.
  Either way this is how it should have been specified
  to start with.
* Added nodelet for scan filtering pipeline
* Pass in public node handle from nodelet to allow for correct topic remapping
* Do not force obsolete C++11 standard
  this breaks with current log4cxx builds which require c++17
* Merge pull request `#163 <https://github.com/eurogroep/laser_filters/issues/163>`_ from rickvanosch/noetic-devel
  Setting STATUS lvl to msgs reg. performance tests
* Setting STATUS lvl to msgs reg. performance tests
  Default level of messages is None/Notice, these are forwarded to stderr, triggering a warning in the build when using catkin build.
* Merge pull request `#158 <https://github.com/eurogroep/laser_filters/issues/158>`_ from LarsJanssenTUe/noetic-devel
  Boxfilter dynamic reconfigure for noetic devel
* Fix(BoxFilter): fix copy mistakes in cfg file
* Feature(BoxFilter): add dynamic reconfigure to box filter
* Merge pull request `#155 <https://github.com/eurogroep/laser_filters/issues/155>`_ from erwinbonsmatopic/improve-static-polygon-filter
  Improve robustness of polygon fetch by static polygon filter
* Improve logging
  Also minor formatting changes.
* Fix build warning
* Make transform time-out configurable
* Do not look for specific time in static filter
* Merge pull request `#157 <https://github.com/eurogroep/laser_filters/issues/157>`_ from erwinbonsmatopic/consistently-publish-polygon
  Consistently publish polygon
* Merge pull request `#153 <https://github.com/eurogroep/laser_filters/issues/153>`_ from JohnTGZ/noetic-devel
  Added examples for InterpolationFilter and LaserScanAngularBoundsFilter
* removed .vscode file
* Merge pull request `#154 <https://github.com/eurogroep/laser_filters/issues/154>`_ from erwinbonsmatopic/add-static-polygon-filter-plugin
  Register static polygon filter as plugin
* Move common logic to base class function
* Add polygon publishing to static filter
* added interpolation filter example
* added angle filter example
* Merge pull request `#149 <https://github.com/eurogroep/laser_filters/issues/149>`_ from JohnTGZ/noetic-devel
  added example config and launch file for scan blob filter
* added example config and launch file for scan blob filter
* Add plugin
* Merge pull request `#145 <https://github.com/eurogroep/laser_filters/issues/145>`_ from erwinbonsmatopic/add-static-polygon-filter
  Add static polygon filter
* Merge pull request `#144 <https://github.com/eurogroep/laser_filters/issues/144>`_ from erwinbonsmatopic/speed-up-shadow-filter
  Speed up shadow filter
* Minor clean-up
  Remove empty destructor.
  Remove unneeded friend declaration.
* Make isShadow overload public and add comments
* Move (co)sine caching to shadows filter
* Replace arrays by vectors
* Rename variables for style consistency
* Reorder methods to group by class
* Remove unnecessary parentheses
* Declare missing variable
* Fix the build
* Fix the build
* Apply review comments
* Split polygn filter into existing, static filters
* Minor changes
* Update unit tests
* Optimize angle_increment change handling
* Fix missing NULL definition
* Improve shadov filter and detector performance
* Merge pull request `#139 <https://github.com/eurogroep/laser_filters/issues/139>`_ from erwinbonsmatopic/split-shadow-filter-headers
  Split shadow filter headers
* Revert some changes to copyright headers
  This addresses review comments in PR `#139 <https://github.com/eurogroep/laser_filters/issues/139>`_
* Merge pull request `#141 <https://github.com/eurogroep/laser_filters/issues/141>`_ from erwinbonsmatopic/create-shadow-filter-tests
  Create shadow filter tests
* Tweak and extend tests
* Add unit and performance tests
* Merge branch 'noetic-devel' into split-shadow-filter-headers
* Merge pull request `#140 <https://github.com/eurogroep/laser_filters/issues/140>`_ from jonbinney/jbinney-catkin-make-on-ci
  Use catkin_make to build and run tests on CI
* Fix checkout path in CI
* Update path to CI script
* Use catkin_make to build and run tests on CI
  Using cmake directly wasn't handling the LD_LIBRARY_PATH correctly when
  running tests, causing them to use the version of the laser_filters
  library from /opt/ros/... instead of the one compiled from source.
* Fix to CI set-up (a workaround)
* Merge pull request `#130 <https://github.com/eurogroep/laser_filters/issues/130>`_ from erwinbonsmatopic/speed-up-speckle-filter
  Speed up speckle filter implementation
* Ensure destructor remains virtual
* Fix build of shadow detector test
* Update headers
* Clean up include statements
* Merge branch 'noetic-devel' into split-shadow-filter-headers
* Split shadow filter and detector header files
* Speed up speckle filter implementation
  Main changes:
  - Do not allocate dynamic memory in each update invocation
  - Speed up loop end-condition checks
  - Algorithm improvements:
  - Perform out-of-range check during initialisation, only once for each
  sample
  - Remove out of bound check from distance window validator.
  Handle this by setting loop end criterion.
* Contributors: Bohdan Yarema, Erwin Bonsma, Jon Binney, Jonathan Binney, Lars, Lucas Walter, Michael Ripperger, Yannick de Hoop, johntgz, rickvanosch, v4hn

1.9.0 (2021-11-06)
------------------
* change_access specifier kinect
* Added nodelet version of scan_to_cloud_filter_chain .
* fix(speckle_filter): Possible segfault when ranges size was smaller than filter window
  formatting
* Lots of fixes to CI
* scan_to_cloud_filter_chain: Make cloud channels configurable
* Fixed naming of laser filter plugins in yaml files
* Add circle sector sharp filter
* Added DynamicReconfigure for RangeFilter
* Added support for laserscanners that spin clockwise
* Added nodelet version of scan_to_cloud_filter_chain .
* Contributors: Arjanboeve, Eric Wiener, Jimmy F. Klarke, Jonathan Binney, Martin Pecka, Rein Appeldoorn, YoshuaNava, renan028, teundeplanque

1.8.11 (2020-06-03)
-------------------
* Merge pull request `#97 <https://github.com/ros-perception/laser_filters/issues/97>`_ from eurogroep/feat/speckle-filter-for-noise-removal
* Merge pull request `#96 <https://github.com/ros-perception/laser_filters/issues/96>`_ from eurogroep/feat/intensity-filter-dynamic-reconfigure-and-optionally-override-intensity-values
  feat(IntensityFilter): Dynamic reconfigure and optionally override intensity
* Merge pull request `#3 <https://github.com/ros-perception/laser_filters/issues/3>`_ from nlimpert/nlimpert/speckle-filter-radius-outlier-merge
  Merge distance based speckle filter with RadiusOutlier removal
* Contributors: Jonathan Binney, Nicolas Limpert, Rein Appeldoorn

1.8.10 (2020-04-07)
-------------------
* radius_outlier_filter: new filter for radius based outlier removal
  Add a new filter to remove measurements that do not have a number of
  neighbors within a certain range.
* Contributors: Jonathan Binney, Nicolas Limpert

1.8.9 (2020-04-05)
------------------
* Bump CMake version to avoid CMP0048 warning
* Polygon filter
* Add dynamic reconfigure for scan shadows filter
* Parameter to remove shadow start point in scan shadows filter
* Contributors: Jonathan Binney, Rein Appeldoorn, Yannick_de_Hoop, ahcorde

1.8.8 (2019-11-07)
------------------
* Merge pull request `#83 <https://github.com/ros-perception/laser_filters/issues/83>`_ from remco-r/indigo-devel
  [fix] when high fidelity true added laser_max_range\_ to projection
* [fix] when high fidelity true added laser_max_range\_ to projection
* Merge pull request `#79 <https://github.com/ros-perception/laser_filters/issues/79>`_ from Jailander/indigo-devel
  Adding invert filter parameter to BOX filter
* Merge pull request `#80 <https://github.com/ros-perception/laser_filters/issues/80>`_ from k-okada/indigo-devel
  Add scan blob filters
* add scan blob filters
* Merge pull request `#72 <https://github.com/ros-perception/laser_filters/issues/72>`_ from ms-iot/windows_port_tests_fix
  [Windows][indigo] Use ${GTEST_LIBRARIES} for more portable gtest library linkage.
* Adding invert filter parameter to BOX filter
* Remove extra changes.
* windows bring up
* Contributors: Jonathan Binney, Kei Okada, Remco, Sean Yen, jailander

1.8.7 (2019-06-13)
------------------
* Merge pull request `#77 <https://github.com/ros-perception/laser_filters/issues/77>`_ from bionade24/indigo-devel
  Removed boost signals from CMakeLists.txt
* Removed boost signals from CMakeLists.txt
  With boost=>1.69 there `signals` isn't available anymore. As it's not necessary, it should be removed to be compatible to all boost versions.
* Merge pull request `#76 <https://github.com/ros-perception/laser_filters/issues/76>`_ from peci1/fix_travis
  Fix Travis and move on to Kinetic and Lunar.
* Fix Travis and move on to Kinetic and Lunar.
* Merge pull request `#73 <https://github.com/ros-perception/laser_filters/issues/73>`_ from peci1/patch-1
  Added error message when the filter chain failed.
* Added error message when the filter chain failed.
* Merge pull request `#62 <https://github.com/ros-perception/laser_filters/issues/62>`_ from at-wat/optimize-shadows-filter
  Reduce computation cost of ScanShadowsFilter
* Merge pull request `#63 <https://github.com/ros-perception/laser_filters/issues/63>`_ from procopiostein/indigo-devel
  set values for variables that could be used uninitialized
* Add some comments to ScanShadowDetector
* set values for variables that could be used uninitialized
* Reduce computation cost of ScanShadowsFilter
  ScanShadowsFilter required a lot of CPU power mainly due to atan2.
  This commit reduces computation cost of the filter.
  * Remove atan2 and directly compare tangent values
  * Add a test to check geometric calculation
* Apply ROS recommended indent style to ScanShadowsFilter
* Contributors: Atsushi Watanabe, Jonathan Binney, Martin Pecka, Oskar Roesler, Procópio Stein

1.8.6 (2018-04-11)
------------------
* Updated deprecated pluginlib macros to avoid warning messages
* Contributors: Jonathan Binney, Nick Lamprianidis

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
