cmake_minimum_required(VERSION 3.0.2)
project(dynamic_occpuancy_map)

add_definitions(-std=c++14 -g -O3 -ftree-vectorize -ffast-math -march=native -lpthread)
#add_definitions(-std=c++14)

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_filters
  sensor_msgs #
  image_transport
)


add_definitions(${PCL_DEFINITIONS})

catkin_package(

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

include_directories(/usr/local/include/munkres)
set(MunkresLIB
        /usr/local/lib/libmunkres.a
        )

add_executable(map_sim_example src/map_sim_example.cpp)
target_link_libraries(map_sim_example ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${MunkresLIB})


