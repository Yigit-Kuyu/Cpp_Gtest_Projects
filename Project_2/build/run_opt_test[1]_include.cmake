if(EXISTS "/home/yck/Desktop/GITHUB/CppTest/Cpp_Gtest_Projects/Project_2/build/run_opt_test[1]_tests.cmake")
  include("/home/yck/Desktop/GITHUB/CppTest/Cpp_Gtest_Projects/Project_2/build/run_opt_test[1]_tests.cmake")
else()
  add_test(run_opt_test_NOT_BUILT run_opt_test_NOT_BUILT)
endif()
