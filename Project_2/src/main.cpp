#include <iostream>
#include "product.h"
#include <gtest/gtest.h>


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv); // Initialize Google Test with command-line arguments

    return RUN_ALL_TESTS(); // Run all the tests
}
