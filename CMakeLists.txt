cmake_minimum_required(VERSION 3.8)
project(tuw_geometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgcodecs imgproc videoio)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/utils.cpp
  src/${PROJECT_NAME}/point2d.cpp
  src/${PROJECT_NAME}/polar2d.cpp
  src/${PROJECT_NAME}/pose2d.cpp
  src/${PROJECT_NAME}/line2d.cpp
  src/${PROJECT_NAME}/plane3d.cpp
  src/${PROJECT_NAME}/command2d.cpp
  src/${PROJECT_NAME}/linesegment2d.cpp
  src/${PROJECT_NAME}/linesegment2d_detector.cpp
  src/${PROJECT_NAME}/figure.cpp
  src/${PROJECT_NAME}/tuw_geometry.cpp
  src/${PROJECT_NAME}/world_scoped_maps.cpp
  src/${PROJECT_NAME}/layered_maps.cpp
  src/${PROJECT_NAME}/layered_figure.cpp
  src/${PROJECT_NAME}/map_handler.cpp
)

target_compile_features(tuw_geometry PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(tuw_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  OpenCV
)

  # Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(tuw_geometry PRIVATE "TUW_GEOMETRY_BUILDING_LIBRARY")

add_subdirectory(src/examples)


install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS tuw_geometry
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_geometry test/test_geometry.cpp)  # Need to link .cpp file under test
  target_include_directories(test_geometry PUBLIC src/)  # Need to specify directories in order to #include from them in the test
  target_link_libraries(test_geometry tuw_geometry)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  install(
    TARGETS test_geometry
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  tuw_geometry
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  OpenCV
)

ament_package()
