cmake_minimum_required(VERSION 3.0.2)
set(ENV{CC} /usr/bin/gcc)
set(ENV{CXX} /usr/bin/g++)

project(wirispro_manager_panel DESCRIPTION "Interactive camera manager for Workswell Wiris Pro model - WirisPro Manager Panel" LANGUAGES CXX C)


#####################
## Project Options ##
#####################

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Force some variables that could be defined in the command line to be written to cache
option(WARNINGS_ARE_ERRORS "Treat warnings as errors"                                  OFF)
option(WARNINGS_ANSI_ISO   "Issue all the mandatory diagnostics listed in C standard"  ON)
option(WARNINGS_EFFCPP     "Issue all the warnings listed in the book of Scot Meyers"  OFF)

if(${WARNINGS_ANSI_ISO})
   add_compile_options(-Wcast-align)
   add_compile_options(-Wpedantic)
   add_compile_options(-Wextra)
else()
   add_compile_options(-Wno-narrowing)
   add_compile_options(-Wno-delete-non-virtual-dtor)
   add_compile_options(-Wno-unnamed-type-template-args)
endif()

if(${WARNINGS_ARE_ERRORS})
   add_compile_options(-Werror)
endif()

if(${WARNINGS_EFFCPP})
   add_compile_options(-Weffc++)
endif()

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra)

IF(NOT DEFINED CMAKE_PREFIX_PATH)
   IF(NOT "$ENV{CMAKE_PREFIX_PATH}" STREQUAL "")
      IF(NOT WIN32)
        STRING(REPLACE ":" ";" CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
      ELSE()
        SET(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
      ENDIF()
   ENDIF()
ENDIF()
   
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_INSTALL_PREFIX}/lib/cmake/)
   
MESSAGE(STATUS "Using CMAKE_PREFIX_PATH - ${CMAKE_PREFIX_PATH}")



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pluginlib
  rosbag
  roscpp
  rviz
  wirispro_manager
  cv_bridge
  gremsy_base
)

# Find OpenCV version 4.2
find_package(OpenCV)
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
   INCLUDE_DIRS   include
   LIBRARIES      ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp pluginlib rviz rosbag wirispro_manager gremsy_base
   DEPENDS        
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
#######################
## Configuring Qt5   ##
#######################
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

#####################################
##        Create library           ##
#####################################
file(GLOB_RECURSE SRCS "src/*.cpp")
file(GLOB_RECURSE HDRS "include/*.h")

file(GLOB_RECURSE UIS   "src/*.ui")
QT5_WRAP_UI(UIS_H ${UIS})

add_library(${PROJECT_NAME} SHARED ${SRCS} ${HDRS} ${UIS_H})
target_include_directories(${PROJECT_NAME} PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>
)

target_include_directories(${PROJECT_NAME} PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${catkin_LIBRARIES} Qt5::Widgets  ${OpenCV_LIBRARIES})
target_compile_options(${PROJECT_NAME} PUBLIC "-Wno-register") # Avoid OGRE deprecaton warnings under C++17

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

INSTALL(FILES wirispro_manager_panel_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)





#########################
## Print configuration ##
#########################

message(STATUS)
message(STATUS "------------ General configuration for - ${PROJECT_NAME} ------------")
message(STATUS)
message(STATUS "WARNINGS_ANSI_ISO   = ${WARNINGS_ANSI_ISO}")
message(STATUS "WARNINGS_ARE_ERRORS = ${WARNINGS_ARE_ERRORS}")
message(STATUS "WARNINGS_EFFCPP     = ${WARNINGS_EFFCPP}")
message(STATUS)
message(STATUS "BUILD_SHARED_LIBS   = ${BUILD_SHARED_LIBS}")
message(STATUS)
message(STATUS "-------------------------- Dependencies --------------------------")
message(STATUS)
message(STATUS "Change a value with: cmake -D<Variable>=<Value>")
message(STATUS)

message(STATUS "--------------------------    OpenCV    --------------------------")

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBRARIES}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")