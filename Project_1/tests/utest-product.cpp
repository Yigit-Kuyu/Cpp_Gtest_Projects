#include "product.h"
#include <gtest/gtest.h>



namespace
{
    TEST(PersonTestSuite, functionA)
    {
        Product p;
        EXPECT_EQ(p.computeA(10, 20), 200); // checks if the computed result is equal to 200.
    }
    TEST(PersonTestSuite, functionB)
    {
        Product p;
        EXPECT_EQ(p.computeB(10, 20, 30), 6000); // checks if the computed result is equal to 6000.
    }

    TEST(PersonTestSuite, not_equal_functionB)
    {
        Product p;
        EXPECT_NE(p.computeB(10, 20, 30), 5000); // checks if the computed result is not equal to 5000.
    }

    TEST(PersonTestSuite, fun_B_greater_funA)
    {
        Product p;
        EXPECT_GT(p.computeB(10, 20, 30), p.computeA(10, 10)); // checks if the computed result of computeB is greater than the computed result of computeA.
    }

}
