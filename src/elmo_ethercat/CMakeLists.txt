add_compile_options(-std=c++11)
cmake_minimum_required(VERSION 3.0.2)
project(elmo_ethercat)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  soem
)
find_package(cmake_modules REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)

include_directories(${Eigen3_INCLUDE_DIRS})

add_definitions(${EIGEN_DEFINITIONS})
add_message_files(
  DIRECTORY msg
  FILES
)
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  LIBRARIES elmo_ethercat elmo_cest_tt testmain
  DEPENDS system_lib Eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}
    /usr/xenomai/include
    /usr/xenomai/include/cobalt 
)

link_directories(
    ${catkin_LIB_DIRS} lib
    /usr/xenomai/lib
)

link_libraries(
    alchemy
    cobalt	
    pthread  
)

execute_process(COMMAND /usr/xenomai/bin/xeno-config --posix --alchemy --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND /usr/xenomai/bin/xeno-config --posix --alchemy --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}${XENO_CFLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${XENO_LDFLAGS}")

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/gene_sku6_elmo.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(main src/main.cpp include/Elmo_EtherCAT.cpp)
add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main ${catkin_LIBRARIES}  ${catkin_LIBRARIES}  ${Eigen3_LIBRARIES})

add_executable(current_control src/current_control.cpp)
add_dependencies(current_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(current_control ${catkin_LIBRARIES} )

