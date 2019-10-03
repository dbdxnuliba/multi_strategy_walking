# 
#  Copyright (C) 2017 IIT-ADVR
#  Author: 
#  email: 
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>
#

# CMake 3.5.1 required 
cmake_minimum_required(VERSION 3.5.1)

# XBot RT Plugin
project(multi_strategy_walking)

# C++11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++1y" COMPILER_SUPPORTS_CXX1Y)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX1Y)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y -fPIC")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++1y support. Please use a different C++ compiler.")
endif()

# Add cmake dir and include custom macros
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroYCMInstallLibrary)

# find required packages
find_package(XCM REQUIRED)
find_package(OpenSoT QUIET)



find_package(Armadillo 5.4 REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})






# find_package(qpOASES QUIET)

# ROS
find_package(catkin QUIET COMPONENTS roscpp)

# XENOMAI
set(ENABLE_XENO CACHE BOOL "ON")
if ( ${ENABLE_XENO} )
   find_package(Xenomai)
endif()


set(LIBRARIES armadillo)

SET(CMAKE_BUILD_TYPE Release)

SET(SRC_PATH  ${PROJECT_SOURCE_DIR}/src/walking)

set(Filters_DIRS ${SRC_PATH}/Filters)
set(IK_DIRS ${SRC_PATH}/IK)
set(LIPM_DIRS ${SRC_PATH}/LIPM)
set(LQR_DIRS ${SRC_PATH}/LQR)
set(ReactiveStep_DIRS ${SRC_PATH}/ReactiveStep)
set(PreviewCtrl_DIRS ${SRC_PATH}/PreviewCtrl)
set(RobotPara_DIRS ${SRC_PATH}/RobotPara)
set(RTControl_DIRS ${SRC_PATH}/RTControl)
set(Stabilizer_DIRS ${SRC_PATH}/Stabilizer)
set(WBS_DIRS ${SRC_PATH}/WBS)
set(ZMPWalk_DIRS ${SRC_PATH}/ZMPWalk)
set(RobotModel_DIRS ${SRC_PATH}/RobotModel)
set(Utils_DIRS ${SRC_PATH}/utils)
set(QP_DIRS ${SRC_PATH}/QP)
set(MPC_DIRS ${SRC_PATH}/MPC)
set(Ext_DIRS ${SRC_PATH}/utils/External)
set(KMP_DIRS ${SRC_PATH}/KMP)

find_package(RBDL REQUIRED)
find_package(urdfdom REQUIRED)


INCLUDE_DIRECTORIES(${SRC_PATH}
                    ${RBDL_INCLUDE_DIR}
                    ${RBDL_URDFReader_INCLUDE_DIR}
                    ${urdfdom_INCLUDE_DIRS}
                    )

FILE(GLOB SOURCES
	${IK_DIRS}/*.cpp
	${RTControl_DIRS}/XBotRTControlClass.cpp
        ${RTControl_DIRS}/MpcRTControlClass.cpp
	${RobotPara_DIRS}/*.cpp
	${Filters_DIRS}/*.cpp
	${Stabilizer_DIRS}/*.cpp
	${WBS_DIRS}/*.cpp
        ${RobotModel_DIRS}/*.cpp
	${QP_DIRS}/*.cpp
	${MPC_DIRS}/MPCClass.cpp
        ${KMP_DIRS}/*.cpp
	${Utils_DIRS}/EiQuadProg/*.cpp
)

ADD_DEFINITIONS( -DROBOT_PARA_YAML_PATH="${SRC_PATH}/RobotPara/" )
# ADD_DEFINITIONS( -DIN_GAZEBO)
ADD_DEFINITIONS( -DREAL_ROBOT)
ADD_DEFINITIONS( -DUSE_XBOT_LOGGER)
ADD_DEFINITIONS( -DUSE_OPENSOT)
ADD_DEFINITIONS( -DUSE_ROS)

SET(ROB_LIB_DIRS ${SRC_PATH}/utils/lib)
LINK_DIRECTORIES(${ROB_LIB_DIRS})


MESSAGE(STATUS "multi_strategy's Source Directory: ${SRC_PATH}")
MESSAGE(STATUS "multi_strategy's Library Directory: ${ROB_LIB_DIRS}")
MESSAGE(STATUS "multi_strategy's Ext Library Directory: ${Ext_DIRS}/lib")
MESSAGE(STATUS "ROS Include Directory: ${ROS_INCLUDE_DIRS}")
MESSAGE(STATUS "ROS Library Directory: ${ROS_LIB_DIRS}")
MESSAGE(STATUS "catkin_LIBRARIES Directory: ${catkin_LIBRARIES}")

# ===========================================================


# include
include_directories(include
                    ${XCM_INCLUDE_DIRS}
                    ${OpenSoT_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
#                     ${qpOASES_INCLUDE_DIRS}
                    )

# lib
add_library(${PROJECT_NAME} SHARED
    src/multi_strategy_walking_rt_plugin.cpp
    ${SOURCES}
)

# set Xenomain flags if Xenomai is found
if ( Xenomai_FOUND )
    set_xeno_flags(${PROJECT_NAME})
endif()

# linking
target_link_libraries(${PROJECT_NAME}   ${XCM_LIBRARIES}
                                        ${OpenSoT_LIBRARIES}
#                                         ${qpOASES_LIBRARIES}
                                        )

target_link_libraries(${PROJECT_NAME}  ${RBDL_LIBRARY}
                                       ${RBDL_URDFReader_LIBRARY}
                                       ${urdfdom_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${LIBRARIES})
                                       
# ROS linking
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES})

#install
ycm_library_install(${PROJECT_NAME} 1 0 0)
