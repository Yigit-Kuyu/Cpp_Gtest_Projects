# $ cmake --version
cmake_minimum_required(VERSION 3.5)

# Set the project name
project (Project_3)


# add googletest subdirectory
add_subdirectory(googletest)

# add header files in the lib
include_directories(lib)

# Create a sources variable with a link to all cpp files to compile
set(SOURCES
    src/main.cpp
    src/product.cpp
    tests/product_test_fixture.cpp
)


enable_testing()
add_executable(run_opt_test ${SOURCES})

target_link_libraries(run_opt_test PRIVATE GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(run_opt_test )
