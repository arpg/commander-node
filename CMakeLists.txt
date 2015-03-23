project(ninja-gui)
cmake_minimum_required(VERSION 2.8)

set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules ${CMAKE_MODULE_PATH} )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

option(JOYSTICK_CONTROL "Enable in order to control ninja with joystick" ON)

find_package( HAL REQUIRED )
find_package( Node REQUIRED )
find_package( Pangolin REQUIRED )
find_package( SceneGraph REQUIRED )
find_package( PbMsgs REQUIRED )
find_package( ZeroMQ REQUIRED )
find_package( ZeroMQPP REQUIRED )
find_package( Boost REQUIRED )
find_package( Protobuf REQUIRED )
include_directories( ${CMAKE_BINARY_DIR} )
PROTOBUF_GENERATE_CPP( PROTO_SRCS PROTO_HDRS NinjaMsgs.proto )

include_directories( ${HAL_INCLUDE_DIRS} )
include_directories( ${Pangolin_INCLUDE_DIRS} )
include_directories( ${SceneGraph_INCLUDE_DIRS} )
include_directories( ${Node_INCLUDE_DIRS} )
include_directories( ${ZeroMQ_INCLUDE_DIRS} )
include_directories( ${ZeroMQPP_INCLUDE_DIRS} )
include_directories( ${PbMsgs_INCLUDE_DIRS} )
include_directories( ${Boost_INCLUDE_DIRS} )

add_subdirectory( Gamepad )
set( HDRS ${PROTO_HDRS} NinjaGUI.h JoystickHandler.h)

if(JOYSTICK_CONTROL)
    set( SRCS JoystickHandler.cpp control-joystick.cpp ${PROTO_SRCS} )
else()
    set( SRCS main.cpp NinjaGUI.cpp JoystickHandler.cpp control-daemon.cpp ${PROTO_SRCS} )
endif()

add_executable( ${PROJECT_NAME} ${HDRS} ${SRCS} )

target_link_libraries( ${PROJECT_NAME}
    Gamepad
    ${HAL_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${SceneGraph_LIBRARIES}
    ${PROTOBUF_LIBRARY}
    ${Node_LIBRARIES}
    ${ZeroMQ_LIBRARIES}
    ${ZeroMQPP_LIBRARIES}
    ${PbMsgs_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${Boost_LIBRARIES}
    )
