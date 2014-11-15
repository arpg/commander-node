project(Gamepad)
cmake_minimum_required(VERSION 2.8)


find_package( IOKit REQUIRED )

#INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${IOKit_INCLUDE_DIRS} )

set( GAMEPAD_HDRS   Gamepad.h
                    EventDispatcher.h
                 )

set( GAMEPAD_SRC    EventDispatcher.cpp
                 )

# test platform type
if(WIN32)
    set( GAMEPAD_SRC ${GAMEPAD_SRC} Gamepad_windows.c )
endif()

if(UNIX)
    if(APPLE)
        set( GAMEPAD_SRC ${GAMEPAD_SRC} Gamepad_macosx.cpp )
        set( GAMEPAD_LIBRARIES ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
    else()
        set( GAMEPAD_SRC ${GAMEPAD_SRC} Gamepad_linux.c )
    endif()
endif()


add_library(Gamepad ${GAMEPAD_HDRS} ${GAMEPAD_SRC} )
target_link_libraries(Gamepad ${GAMEPAD_LIBRARIES} ${IOKit_LIBRARIES} )
