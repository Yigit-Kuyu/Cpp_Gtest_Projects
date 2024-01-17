# CMake generated Testfile for 
# Source directory: /home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5
# Build directory: /home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/build/mpc_lateral_controller
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_lateral_controller "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_ros/cmake/run_test_isolated.py" "/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/build/mpc_lateral_controller/test_results/mpc_lateral_controller/test_lateral_controller.gtest.xml" "--package-name" "mpc_lateral_controller" "--output-file" "/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/build/mpc_lateral_controller/ament_cmake_gtest/test_lateral_controller.txt" "--command" "/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/build/mpc_lateral_controller/test_lateral_controller" "--gtest_output=xml:/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/build/mpc_lateral_controller/test_results/mpc_lateral_controller/test_lateral_controller.gtest.xml")
set_tests_properties(test_lateral_controller PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/build/mpc_lateral_controller/test_lateral_controller" TIMEOUT "60" WORKING_DIRECTORY "/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/build/mpc_lateral_controller" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/opt/ros/humble/share/ament_cmake_ros/cmake/ament_add_ros_isolated_gtest.cmake;33;ament_add_gtest;/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/CMakeLists.txt;50;ament_add_ros_isolated_gtest;/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/CMakeLists.txt;0;")
add_test(copyright "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/build/mpc_lateral_controller/test_results/mpc_lateral_controller/copyright.xunit.xml" "--package-name" "mpc_lateral_controller" "--output-file" "/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/build/mpc_lateral_controller/ament_copyright/copyright.txt" "--command" "/opt/ros/humble/bin/ament_copyright" "--xunit-file" "/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/build/mpc_lateral_controller/test_results/mpc_lateral_controller/copyright.xunit.xml")
set_tests_properties(copyright PROPERTIES  LABELS "copyright;linter" TIMEOUT "200" WORKING_DIRECTORY "/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_copyright/cmake/ament_copyright.cmake;51;ament_add_test;/opt/ros/humble/share/ament_cmake_copyright/cmake/ament_cmake_copyright_lint_hook.cmake;22;ament_copyright;/opt/ros/humble/share/ament_cmake_copyright/cmake/ament_cmake_copyright_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/opt/ros/humble/share/ament_cmake_auto/cmake/ament_auto_package.cmake;102;ament_package;/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/CMakeLists.txt;54;ament_auto_package;/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/CMakeLists.txt;0;")
add_test(cppcheck "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/build/mpc_lateral_controller/test_results/mpc_lateral_controller/cppcheck.xunit.xml" "--package-name" "mpc_lateral_controller" "--output-file" "/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/build/mpc_lateral_controller/ament_cppcheck/cppcheck.txt" "--command" "/opt/ros/humble/bin/ament_cppcheck" "--xunit-file" "/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/build/mpc_lateral_controller/test_results/mpc_lateral_controller/cppcheck.xunit.xml" "--include_dirs" "/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/autoware_auto_control_msgs/include/autoware_auto_control_msgs" "/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/mpc_lateral_controller/include")
set_tests_properties(cppcheck PROPERTIES  LABELS "cppcheck;linter" TIMEOUT "300" WORKING_DIRECTORY "/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_cppcheck/cmake/ament_cppcheck.cmake;66;ament_add_test;/opt/ros/humble/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;87;ament_cppcheck;/opt/ros/humble/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/opt/ros/humble/share/ament_cmake_auto/cmake/ament_auto_package.cmake;102;ament_package;/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/CMakeLists.txt;54;ament_auto_package;/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/CMakeLists.txt;0;")
add_test(lint_cmake "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/build/mpc_lateral_controller/test_results/mpc_lateral_controller/lint_cmake.xunit.xml" "--package-name" "mpc_lateral_controller" "--output-file" "/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/build/mpc_lateral_controller/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/humble/bin/ament_lint_cmake" "--xunit-file" "/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/build/mpc_lateral_controller/test_results/mpc_lateral_controller/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_lint_cmake.cmake;47;ament_add_test;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;21;ament_lint_cmake;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/opt/ros/humble/share/ament_cmake_auto/cmake/ament_auto_package.cmake;102;ament_package;/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/CMakeLists.txt;54;ament_auto_package;/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/CMakeLists.txt;0;")
add_test(xmllint "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/build/mpc_lateral_controller/test_results/mpc_lateral_controller/xmllint.xunit.xml" "--package-name" "mpc_lateral_controller" "--output-file" "/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/build/mpc_lateral_controller/ament_xmllint/xmllint.txt" "--command" "/opt/ros/humble/bin/ament_xmllint" "--xunit-file" "/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/build/mpc_lateral_controller/test_results/mpc_lateral_controller/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_xmllint.cmake;50;ament_add_test;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;18;ament_xmllint;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/opt/ros/humble/share/ament_cmake_auto/cmake/ament_auto_package.cmake;102;ament_package;/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/CMakeLists.txt;54;ament_auto_package;/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/CMakeLists.txt;0;")
subdirs("gtest")