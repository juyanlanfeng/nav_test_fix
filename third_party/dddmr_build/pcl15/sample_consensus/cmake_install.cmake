# Install script for directory: /home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus

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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_sample_consensus" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_sample_consensus.so.1.15.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_sample_consensus.so.1.15"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/home/rainple/nav_test/third_party/dddmr_pcl15/lib")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_sample_consensus.so.1.15.0"
    "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_sample_consensus.so.1.15"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_sample_consensus.so.1.15.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_sample_consensus.so.1.15"
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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_sample_consensus" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_sample_consensus.so")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_sample_consensus" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/rainple/nav_test/third_party/dddmr_build/pcl15/sample_consensus/pcl_sample_consensus.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_sample_consensus" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/sample_consensus" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/lmeds.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/method_types.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/mlesac.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/model_types.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/msac.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/ransac.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/rmsac.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/rransac.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/prosac.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/sac.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/sac_model.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/sac_model_circle.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/sac_model_circle3d.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/sac_model_cylinder.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/sac_model_cone.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/sac_model_line.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/sac_model_stick.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/sac_model_normal_parallel_plane.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/sac_model_normal_plane.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/sac_model_normal_sphere.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/sac_model_parallel_line.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/sac_model_parallel_plane.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/sac_model_perpendicular_plane.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/sac_model_plane.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/sac_model_registration.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/sac_model_registration_2d.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/sac_model_sphere.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/sac_model_ellipse3d.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/sac_model_torus.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_sample_consensus" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/sample_consensus/impl" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/impl/lmeds.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/impl/mlesac.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/impl/msac.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/impl/ransac.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/impl/rmsac.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/impl/rransac.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/impl/prosac.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/impl/sac_model_circle.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/impl/sac_model_circle3d.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/impl/sac_model_cylinder.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/impl/sac_model_cone.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/impl/sac_model_line.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/impl/sac_model_stick.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/impl/sac_model_normal_parallel_plane.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/impl/sac_model_normal_plane.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/impl/sac_model_normal_sphere.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/impl/sac_model_parallel_line.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/impl/sac_model_parallel_plane.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/impl/sac_model_perpendicular_plane.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/impl/sac_model_plane.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/impl/sac_model_registration.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/impl/sac_model_registration_2d.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/impl/sac_model_sphere.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/impl/sac_model_ellipse3d.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/sample_consensus/include/pcl/sample_consensus/impl/sac_model_torus.hpp"
    )
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/rainple/nav_test/third_party/dddmr_build/pcl15/sample_consensus/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
