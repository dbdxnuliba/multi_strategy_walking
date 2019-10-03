project(xddp_console)

################################################################
#    Coman project header. Don't touch unless you know how     # 
################################################################
cmake_minimum_required(VERSION 2.8)

# Add cmake dir and include custom macros
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# XENOMAI
set(ENABLE_XENO CACHE BOOL "ON")
if ( ${ENABLE_XENO} )
   find_package(Xenomai)
endif()

##################################################################
#    Project files.  Adjust to fit your project.                 #
##################################################################

add_definitions("-std=c++0x")

set(SOURCES
xddp_thread.cpp
)

set(LIBRARIES
m
dl
stdc++
)

add_executable(xddp_console ${SOURCES})

# set Xenomain flags if Xenomai is found
if ( Xenomai_FOUND )
    set_xeno_flags(xddp_console)
endif()

target_link_libraries(xddp_console ${LIBRARIES})
install(TARGETS xddp_console DESTINATION bin)


