# tuw_geometry
Classes to handle and visualize 2D and 3D objects such as points, poses, lines as well as a plot windows for debugging.



# doxygen
```
cd tuw_geometry
doxygen Doxyfile
```
# Build
## GeographicLib
The `geo_map.cpp` depends on the GeographicLib. If you cannot install it or if you like to go on without the __GeographicLib__ you have to remove the __geographiclib__ entries from the `package.xml` as well as the lines pointing to  `geo_map.cpp` and the `GeographicLib` from the CMakeFile.txt.

## Test
Unittests can be run by
```
colcon build --packages-select tuw_geometry --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug --cmake-args -DBUILD_TESTING=true
run tuw_geometry test_geometry
```