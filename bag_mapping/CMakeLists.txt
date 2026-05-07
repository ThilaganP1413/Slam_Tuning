cmake_minimum_required(VERSION 3.8)
project(bag_mapping)

find_package(ament_cmake REQUIRED)

# Install Python script as an executable node
install(PROGRAMS
  src/tf_and_odom.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tf_and_odom
)

install(PROGRAMS
  src/static_tf_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME static_tf_publisher
)

install(PROGRAMS
  src/slamware_tf_pub.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME slamware_tf_pub
)

install(PROGRAMS
  src/cartographer_odom_pub.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cartographer_odom_pub
)

install(PROGRAMS
  src/odom_pub.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME odom_pub
)

# Install launch files
install(DIRECTORY
  launch
  config
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()