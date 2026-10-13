cmake_minimum_required(VERSION 3.0.2)
project(turtlesim_circle)

find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs std_msgs turtlesim)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/circle_controller.py
  scripts/analyze_bag.py
  scripts/generate_demo_bag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests/test_motion.py)
  catkin_add_nosetests(tests/test_controller.py)
endif()
