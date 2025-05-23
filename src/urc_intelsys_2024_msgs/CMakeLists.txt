cmake_minimum_required(VERSION 3.8)
project(urc_intelsys_2024_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# generate messages
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CART.msg"
  "msg/GPS.msg"
  "msg/GPSNavDetails.msg"
  "msg/ObjectNavDetails.msg"
  "msg/PolarCoordinate.msg"
  "msg/Task.msg"
  "msg/TGPS.msg"
  "msg/TGPSNavDetails.msg"
  "msg/TPolarCoordinate.msg"
  "msg/TTask.msg"
  # services
  "srv/GeoToCart.srv"
  "srv/GPSDistance.srv"

  DEPENDENCIES std_msgs
)

ament_package()
