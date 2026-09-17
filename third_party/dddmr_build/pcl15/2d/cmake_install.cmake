# Install script for directory: /home/rainple/nav_test/third_party/dddmr_sources/pcl/2d

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/rainple/nav_test/third_party/dddmr_pcl15")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_2d" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/rainple/nav_test/third_party/dddmr_build/pcl15/2d/pcl_2d.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_2d" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/2d" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/2d/include/pcl/2d/convolution.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/2d/include/pcl/2d/kernel.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/2d/include/pcl/2d/edge.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/2d/include/pcl/2d/morphology.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_2d" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/2d/impl" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/2d/include/pcl/2d/impl/convolution.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/2d/include/pcl/2d/impl/kernel.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/2d/include/pcl/2d/impl/edge.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/2d/include/pcl/2d/impl/morphology.hpp"
    )
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/rainple/nav_test/third_party/dddmr_build/pcl15/2d/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
