# CMake generated Testfile for 
# Source directory: /home/odroid/MAV-Project/px4_ws/src/catkin
# Build directory: /home/odroid/MAV-Project/px4_ws/build/catkin
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
ADD_TEST(_ctest_catkin_nosetests_test.local_tests "/home/odroid/MAV-Project/px4_ws/build/catkin/catkin_generated/env_cached.sh" "/usr/bin/python" "/home/odroid/MAV-Project/px4_ws/src/catkin/cmake/test/run_tests.py" "/home/odroid/MAV-Project/px4_ws/build/catkin/test_results/catkin/nosetests-test.local_tests.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/odroid/MAV-Project/px4_ws/build/catkin/test_results/catkin" "/usr/bin/nosetests-2.7 -P --process-timeout=60 --where=/home/odroid/MAV-Project/px4_ws/src/catkin/test/local_tests --with-xunit --xunit-file=/home/odroid/MAV-Project/px4_ws/build/catkin/test_results/catkin/nosetests-test.local_tests.xml")
ADD_TEST(_ctest_catkin_nosetests_test.unit_tests "/home/odroid/MAV-Project/px4_ws/build/catkin/catkin_generated/env_cached.sh" "/usr/bin/python" "/home/odroid/MAV-Project/px4_ws/src/catkin/cmake/test/run_tests.py" "/home/odroid/MAV-Project/px4_ws/build/catkin/test_results/catkin/nosetests-test.unit_tests.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/odroid/MAV-Project/px4_ws/build/catkin/test_results/catkin" "/usr/bin/nosetests-2.7 -P --process-timeout=60 --where=/home/odroid/MAV-Project/px4_ws/src/catkin/test/unit_tests --with-xunit --xunit-file=/home/odroid/MAV-Project/px4_ws/build/catkin/test_results/catkin/nosetests-test.unit_tests.xml")
SUBDIRS(gtest)
