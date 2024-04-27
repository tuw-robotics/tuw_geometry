
add_executable(example_figure main_example_figure.cpp)
target_include_directories(example_figure PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(example_figure PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(example_figure
  rclcpp::rclcpp
  ${PROJECT_NAME}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${OpenCV_LIBS})

install(TARGETS example_figure
  DESTINATION lib/${PROJECT_NAME}
)


set(EXAMPLE_GEO_MAP 0 CACHE BOOL "build geo map sample. GeographicLib is requiered!")
if(EXAMPLE_GEO_MAP)
# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib QUIET)
if(GeographicLib_FOUND)
  message(STATUS "GeographicLib found: building geo_map!")
else()
  message(ERROR "GeographicLib NOT found: not building geo_map!")
endif()


add_executable(example_geo_map main_example_geo_map.cpp)
target_include_directories(example_geo_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(example_geo_map PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(example_geo_map
  rclcpp::rclcpp
  ${PROJECT_NAME}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${OpenCV_LIBS}
)

#target_link_libraries(example_geo_map GeographicLib)

install(TARGETS example_geo_map
  DESTINATION lib/${PROJECT_NAME}
)
endif()