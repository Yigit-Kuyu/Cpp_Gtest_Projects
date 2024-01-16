#include "gtest/gtest.h"
#include "declerations.h"
#include "definitions.cpp"


class YcKTest : public ::testing::Test {

protected:
    YcK<int> object1;

protected:
    virtual void TearDown() {}

    virtual void SetUp() {}

};

TEST_F(YcKTest, TrueFalse) {
    ASSERT_EQ(object1.empty(1), true); // checks whether the YcK object (object1) is empty
    ASSERT_EQ(object1.pop(2), false); //  checks if popping an element from the second stack (flag 2) on an empty stack returns false.
    object1.push(1, 1); // pushes an element 1 onto the first stack (flag 1)
    ASSERT_EQ(object1.empty(1), false); // checks if the stack is empty.
    ASSERT_EQ(object1.empty(2), true); //  checks whether the second stack (flag 2) is empty.
    // These operations push elements onto both the first and second stacks:
    object1.push(2, 1);
    object1.push(1, 2);
    object1.push(2, 2);

    ASSERT_EQ(object1.push(3, 2), true); // To pass, expects true
    ASSERT_EQ(object1.push(4,2),false); // // To pass, expects false
    int e = 3;
    ASSERT_EQ(object1.top(e, 1),true);
    ASSERT_EQ(e,2);
    ASSERT_EQ(object1.top(e, 2), true);
    ASSERT_EQ(e, 3);
    ASSERT_EQ(object1.pop(2), true);
    ASSERT_EQ(object1.top(e,2),true);
    ASSERT_EQ(e, 2);
}


int main() {
    ::testing::InitGoogleTest();
    return RUN_ALL_TESTS();
}
