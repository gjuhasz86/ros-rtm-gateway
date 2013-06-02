cmake_minimum_required(VERSION 2.8.3)
project(ros_rtm_gateway)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  /usr/include/openrtm-1.1
  /usr/include/openrtm-1.1/rtm/idl
  /usr/include/coil-1.1

)

add_library(ros_rtm_gateway_coutlong src/CoutLong.cpp)
add_library(ros_rtm_gateway_coutstring src/CoutString.cpp)
#add_library(ros_rtm_gateway src/Gateway.cpp)

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
#add_dependencies(talker ros_rtm_gateway_gencpp)

add_executable(talkerstring src/talkerString.cpp)
target_link_libraries(talkerstring ${catkin_LIBRARIES})
add_dependencies(talkerstring ros_rtm_gateway_gencpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
#add_dependencies(listener ros_rtm_gateway_gencpp)

add_executable(HybridComp src/HybridComp.cpp)
target_link_libraries(HybridComp ${catkin_LIBRARIES} RTC coil uuid dl pthread omniORB4 omnithread omniDynamic4)

add_executable(GatewayComp1 src/GatewayComp1.cpp)
target_link_libraries(GatewayComp1 ${catkin_LIBRARIES} RTC coil uuid dl pthread omniORB4 omnithread omniDynamic4)

#add_executable(MultiComp src/MultiComp.cpp)
#target_link_libraries(MultiComp ${catkin_LIBRARIES} ros_rtm_gateway_coutlong ros_rtm_gateway_coutstring RTC coil uuid dl pthread omniORB4 omnithread omniDynamic4)

add_executable(test src/test.cpp)
target_link_libraries(test ${catkin_LIBRARIES})
