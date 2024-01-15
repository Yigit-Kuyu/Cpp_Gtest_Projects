// product_test_fixture.cpp
#include "product_test_fixture.h"


#  If the condition is true, the test passes

TEST_F(ProductTestFixture, functionA)
{
    EXPECT_EQ(p.computeA(10, 20), 200); // expect equal
}

TEST_F(ProductTestFixture, functionB)
{
    EXPECT_EQ(p.computeB(10, 20, 30), 999000); // expect equal
}

TEST_F(ProductTestFixture, not_equal_functionB)
{
    EXPECT_NE(p.computeB(10, 20, 30), 5000); // expect not equal
}

TEST_F(ProductTestFixture, fun_B_greater_funA)
{
    EXPECT_GT(p.computeB(10, 20, 30), p.computeA(10, 10));  // expect greater than
}
