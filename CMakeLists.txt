cmake_minimum_required(VERSION 3.5)
project(pi_robot)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(CMAKE_CXX_FLAGS  "$ENV{CXXFLAGS} -O3")

# libfacedetection config
SET(fdt_base_dir   ${PROJECT_SOURCE_DIR})
SET(fdt_src_dir    ${fdt_base_dir}/src/libfacedetection)
SET(fdt_inc_dir    ${fdt_base_dir}/include/libfacedetection)

SET(fdt_lib_name   facedetection)
SET(fdt_lib_static ${fdt_lib_name})
SET(fdt_lib_shared ${fdt_lib_name}_shared)

FILE(GLOB_RECURSE fdt_source_files ${fdt_src_dir}/*.cpp)
LIST(SORT         fdt_source_files)

# ultraface config
SET(uf_src_dir ${PROJECT_SOURCE_DIR}/src/ultraface)
SET(uf_inc_dir ${PROJECT_SOURCE_DIR}/include/ultraface)

SET(uf_lib_name   ultraface)
SET(uf_lib_static ${uf_lib_name})
SET(uf_depend_lib ${PROJECT_SOURCE_DIR}/lib/ultraface/libMNN.so)

FILE(GLOB_RECURSE uf_source_files ${uf_src_dir}/*.cpp)
LIST(SORT         uf_source_files)

message("using avx2")
add_definitions(-D_ENABLE_AVX2)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx2 -mfma")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
        sensor_msgs
        image_transport
        cv_bridge
        )

## System dependencies are found with CMake's conventions

find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   SrvTrigger.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES pi_robot
  CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS k4a
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/pi_robot
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${fdt_inc_dir} ${uf_inc_dir}
)

# Create a static library (.a)
ADD_LIBRARY(${fdt_lib_static} STATIC ${fdt_source_files})
ADD_LIBRARY(${uf_lib_static} STATIC ${uf_source_files})
target_link_libraries(${uf_lib_static} ${uf_depend_lib})
add_library(photoComposer STATIC src/photoComposer.cpp)
target_link_libraries(photoComposer ${OpenCV_LIBS} ${fdt_lib_static} ${uf_lib_static})

# Create a shared library (.so)
ADD_LIBRARY(${fdt_lib_shared} SHARED ${fdt_source_files})
SET_TARGET_PROPERTIES(${fdt_lib_shared} PROPERTIES OUTPUT_NAME "${fdt_lib_name}")
SET_TARGET_PROPERTIES(${fdt_lib_shared} PROPERTIES PREFIX "lib")

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/pi_robot_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

add_executable(image_subscriber src/image_subscriber.cpp)
add_dependencies(image_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_subscriber ${catkin_LIBRARIES} photoComposer)

add_executable(myTest src/test.cpp)
target_link_libraries(myTest photoComposer)