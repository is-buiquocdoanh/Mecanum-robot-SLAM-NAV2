# CMake generated Testfile for 
# Source directory: /home/raspi/ros2_ws/src/YDLidar-SDK-master/python
# Build directory: /home/raspi/ros2_ws/src/YDLidar-SDK-master/build/python
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(ydlidar_py_test "/usr/bin/python3.10" "/home/raspi/ros2_ws/src/YDLidar-SDK-master/python/test/pytest.py")
set_tests_properties(ydlidar_py_test PROPERTIES  ENVIRONMENT "PYTHONPATH=/home/raspi/ros2_ws/build/motor_control:/home/raspi/ros2_ws/build/teleop_twist_keyboard:/home/raspi/ros2_ws/install/teleop_twist_keyboard/lib/python3.10/site-packages:/home/raspi/ros2_ws/install/motor_control/lib/python3.10/site-packages:/home/raspi/ros2_ws/install/message_pkg/local/lib/python3.10/dist-packages:/home/raspi/ros2_ws/build/launch_pkg:/home/raspi/ros2_ws/install/launch_pkg/lib/python3.10/site-packages:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/home/raspi/ros2_ws/src/YDLidar-SDK-master/build/python" _BACKTRACE_TRIPLES "/home/raspi/ros2_ws/src/YDLidar-SDK-master/python/CMakeLists.txt;42;add_test;/home/raspi/ros2_ws/src/YDLidar-SDK-master/python/CMakeLists.txt;0;")
subdirs("examples")
