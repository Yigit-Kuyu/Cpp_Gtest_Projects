# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/yck/Desktop/GITHUB/CppTest/Cpp_Gtest_Projects/Project_4

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yck/Desktop/GITHUB/CppTest/Cpp_Gtest_Projects/Project_4/build

# Include any dependencies generated for this target.
include CMakeFiles/run_opt_test.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/run_opt_test.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/run_opt_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/run_opt_test.dir/flags.make

CMakeFiles/run_opt_test.dir/test/test.cpp.o: CMakeFiles/run_opt_test.dir/flags.make
CMakeFiles/run_opt_test.dir/test/test.cpp.o: ../test/test.cpp
CMakeFiles/run_opt_test.dir/test/test.cpp.o: CMakeFiles/run_opt_test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yck/Desktop/GITHUB/CppTest/Cpp_Gtest_Projects/Project_4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/run_opt_test.dir/test/test.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/run_opt_test.dir/test/test.cpp.o -MF CMakeFiles/run_opt_test.dir/test/test.cpp.o.d -o CMakeFiles/run_opt_test.dir/test/test.cpp.o -c /home/yck/Desktop/GITHUB/CppTest/Cpp_Gtest_Projects/Project_4/test/test.cpp

CMakeFiles/run_opt_test.dir/test/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/run_opt_test.dir/test/test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yck/Desktop/GITHUB/CppTest/Cpp_Gtest_Projects/Project_4/test/test.cpp > CMakeFiles/run_opt_test.dir/test/test.cpp.i

CMakeFiles/run_opt_test.dir/test/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/run_opt_test.dir/test/test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yck/Desktop/GITHUB/CppTest/Cpp_Gtest_Projects/Project_4/test/test.cpp -o CMakeFiles/run_opt_test.dir/test/test.cpp.s

# Object files for target run_opt_test
run_opt_test_OBJECTS = \
"CMakeFiles/run_opt_test.dir/test/test.cpp.o"

# External object files for target run_opt_test
run_opt_test_EXTERNAL_OBJECTS =

run_opt_test: CMakeFiles/run_opt_test.dir/test/test.cpp.o
run_opt_test: CMakeFiles/run_opt_test.dir/build.make
run_opt_test: lib/libgtest_main.a
run_opt_test: lib/libgtest.a
run_opt_test: CMakeFiles/run_opt_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yck/Desktop/GITHUB/CppTest/Cpp_Gtest_Projects/Project_4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable run_opt_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/run_opt_test.dir/link.txt --verbose=$(VERBOSE)
	/usr/bin/cmake -D TEST_TARGET=run_opt_test -D TEST_EXECUTABLE=/home/yck/Desktop/GITHUB/CppTest/Cpp_Gtest_Projects/Project_4/build/run_opt_test -D TEST_EXECUTOR= -D TEST_WORKING_DIR=/home/yck/Desktop/GITHUB/CppTest/Cpp_Gtest_Projects/Project_4/build -D TEST_EXTRA_ARGS= -D TEST_PROPERTIES= -D TEST_PREFIX= -D TEST_SUFFIX= -D TEST_FILTER= -D NO_PRETTY_TYPES=FALSE -D NO_PRETTY_VALUES=FALSE -D TEST_LIST=run_opt_test_TESTS -D CTEST_FILE=/home/yck/Desktop/GITHUB/CppTest/Cpp_Gtest_Projects/Project_4/build/run_opt_test[1]_tests.cmake -D TEST_DISCOVERY_TIMEOUT=5 -D TEST_XML_OUTPUT_DIR= -P /usr/share/cmake-3.22/Modules/GoogleTestAddTests.cmake

# Rule to build all files generated by this target.
CMakeFiles/run_opt_test.dir/build: run_opt_test
.PHONY : CMakeFiles/run_opt_test.dir/build

CMakeFiles/run_opt_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/run_opt_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/run_opt_test.dir/clean

CMakeFiles/run_opt_test.dir/depend:
	cd /home/yck/Desktop/GITHUB/CppTest/Cpp_Gtest_Projects/Project_4/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yck/Desktop/GITHUB/CppTest/Cpp_Gtest_Projects/Project_4 /home/yck/Desktop/GITHUB/CppTest/Cpp_Gtest_Projects/Project_4 /home/yck/Desktop/GITHUB/CppTest/Cpp_Gtest_Projects/Project_4/build /home/yck/Desktop/GITHUB/CppTest/Cpp_Gtest_Projects/Project_4/build /home/yck/Desktop/GITHUB/CppTest/Cpp_Gtest_Projects/Project_4/build/CMakeFiles/run_opt_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/run_opt_test.dir/depend

