# $ cmake --version
cmake_minimum_required(VERSION 3.5)

# Set the project name
project (Project_4)


# add googletest subdirectory
add_subdirectory(googletest)



# Create a sources variable with a link to all cpp files to compile
set(SOURCES
    test/test.cpp
)

# add header files in the lib
include_directories(lib)
include_directories(src)
link_directories(src)
link_directories(lib)


enable_testing()
add_executable(run_opt_test ${SOURCES})

# Link against GoogleTest
target_link_libraries(run_opt_test PRIVATE GTest::gtest_main)





include(GoogleTest)
gtest_discover_tests(run_opt_test)
