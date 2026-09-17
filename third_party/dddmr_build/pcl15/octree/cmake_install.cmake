# Install script for directory: /home/rainple/nav_test/third_party/dddmr_sources/pcl/octree

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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_octree" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_octree.so.1.15.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_octree.so.1.15"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/home/rainple/nav_test/third_party/dddmr_pcl15/lib")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_octree.so.1.15.0"
    "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_octree.so.1.15"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_octree.so.1.15.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_octree.so.1.15"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib:"
           NEW_RPATH "/home/rainple/nav_test/third_party/dddmr_pcl15/lib")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_octree" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_octree.so")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_octree" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/rainple/nav_test/third_party/dddmr_build/pcl15/octree/pcl_octree.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_octree" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/octree" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/octree/include/pcl/octree/octree_base.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/octree/include/pcl/octree/octree_container.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/octree/include/pcl/octree/octree_impl.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/octree/include/pcl/octree/octree_nodes.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/octree/include/pcl/octree/octree_key.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/octree/include/pcl/octree/octree_pointcloud_density.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/octree/include/pcl/octree/octree_pointcloud_occupancy.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/octree/include/pcl/octree/octree_pointcloud_singlepoint.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/octree/include/pcl/octree/octree_pointcloud_pointvector.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/octree/include/pcl/octree/octree_pointcloud_changedetector.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/octree/include/pcl/octree/octree_pointcloud_voxelcentroid.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/octree/include/pcl/octree/octree_pointcloud.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/octree/include/pcl/octree/octree_iterator.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/octree/include/pcl/octree/octree_search.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/octree/include/pcl/octree/octree.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/octree/include/pcl/octree/octree2buf_base.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/octree/include/pcl/octree/octree_pointcloud_adjacency.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/octree/include/pcl/octree/octree_pointcloud_adjacency_container.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_octree" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/octree/impl" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/octree/include/pcl/octree/impl/octree_base.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/octree/include/pcl/octree/impl/octree_pointcloud.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/octree/include/pcl/octree/impl/octree2buf_base.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/octree/include/pcl/octree/impl/octree_iterator.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/octree/include/pcl/octree/impl/octree_search.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/octree/include/pcl/octree/impl/octree_pointcloud_voxelcentroid.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/octree/include/pcl/octree/impl/octree_pointcloud_adjacency.hpp"
    )
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/rainple/nav_test/third_party/dddmr_build/pcl15/octree/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
