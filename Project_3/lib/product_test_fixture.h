// product_test_fixture.h
#ifndef PRODUCT_TEST_FIXTURE
#define PRODUCT_TEST_FIXTURE

#include <gtest/gtest.h>
#include "product.h"

class ProductTestFixture : public ::testing::Test
{
protected:
    Product p; // Create an instance of the Product class for testing
};

#endif // PRODUCT_TEST_FIXTURE

