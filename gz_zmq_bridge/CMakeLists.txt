cmake_minimum_required(VERSION 3.10.2)
project(gz_zmq_camera_bridge)

# Find the standard Gazebo packages
find_package(gz-plugin2 REQUIRED)
find_package(gz-transport12 REQUIRED)
find_package(gz-msgs9 REQUIRED)

# --- CORRECTED ZEROMQ FIND BLOCK ---
# First, find the pkg-config tool itself
find_package(PkgConfig REQUIRED)

# Now, use pkg-config to find the ZeroMQ library and set up variables
pkg_check_modules(ZMQ REQUIRED IMPORTED_TARGET libzmq)
# --- END CORRECTION ---

add_library(GzZmqCameraBridge SHARED GzZmqCameraBridge.cc)

# Link all the found libraries to your plugin
target_link_libraries(GzZmqCameraBridge
  PRIVATE
    gz-plugin2::gz-plugin2
    gz-transport12::gz-transport12
    gz-msgs9::gz-msgs9
    PkgConfig::ZMQ # Use the target created by pkg_check_modules
)