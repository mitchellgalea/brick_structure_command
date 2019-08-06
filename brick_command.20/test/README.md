This will be a small guide to my approach to testing:

Firtly for this approach to testing to be most effective, libraries with classes that are decoupled from ros functionality are most useful. That way easy test cases for classes and functions to be set up.

First add the following to CMakeLists.txt

## Add gtest based cpp test target and link libraries
catkin_add_gtest(name-of-test test/test_file_name.cpp)
if(TARGET name-of-test)
    target_link_libraries(name-of-test ${catkin_LIBRARIES} libraries-used)
    message ( STATUS " catkin_LIBRARIES ${catkin_LIBRARIES}")
endif()

refer to brick_structure-tests.cpp and https://github.com/google/googletest for extensive documentation and the various test types that can be used,
ive tried to make separate tests for functions within each class

to run the tests:
1. Navigate to catkin_ws
2. Call catkin_make run_tests_package_name

This will build the tests and run them 
