### Aim
The object is to independently test the control methods in AW Universe, with ROS2 Humble and C++ 17. The related tests are found in the test folder.

### Implementation

 ```

~/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/autoware_cmake$ source /opt/ros/humble/setup.bash
~/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/autoware_cmake$ colcon build
~/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/autoware_auto_msgs/autoware_auto_control_msgs$ colcon build
~/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5$ colcon test console_cohesion+ --packages-select mpc_lateral_controller

 ```

*Note*: Don't forget to change the path in CMakeLists.txt in Project_5 folder according to your path specification.
