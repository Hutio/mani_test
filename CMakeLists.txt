cmake_minimum_required(VERSION 3.0.2)
project(mani_test)


find_package(catkin REQUIRED COMPONENTS message_generation std_msgs rospy)

add_message_files(DIRECTORY msg FILES Mdata.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS rospy std_msgs)

catkin_install_python(PROGRAMS
  scripts/mani_subscriber.py
  scripts/mani_publisher.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
