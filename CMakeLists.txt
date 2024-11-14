cmake_minimum_required(VERSION 3.8)
project(lanelet_unknown_filter)

find_package(autoware_cmake REQUIRED)
find_package(Boost REQUIRED)
autoware_package()

include_directories(
  SYSTEM
  include
  ${Boost_INCLUDE_DIRS}
)

# target_link_libraries(lanelet_unknown_filter
#   ${Boost_LIBRARIES}
# )

ament_auto_add_executable(lanelet_unknown_filter
  src/lanelet_unknown_filter.cpp
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)