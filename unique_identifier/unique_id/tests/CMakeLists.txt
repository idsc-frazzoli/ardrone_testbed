### Unit tests
#
#   Only configured when CATKIN_ENABLE_TESTING is true.

# C++ gtests
catkin_add_gtest(test_unique_id test_unique_id.cpp second_test_unique_id.cpp)
add_dependencies(test_unique_id ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_unique_id
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES})

# Python nose tests
catkin_add_nosetests(test_unique_id.py)
