# Install script for directory: /home/rainple/nav_test/third_party/dddmr_sources/pcl/features

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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_features" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_features.so.1.15.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_features.so.1.15"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/home/rainple/nav_test/third_party/dddmr_pcl15/lib")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_features.so.1.15.0"
    "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_features.so.1.15"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_features.so.1.15.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_features.so.1.15"
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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_features" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_features.so")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_features" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/rainple/nav_test/third_party/dddmr_build/pcl15/features/pcl_features.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_features" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/features" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/board.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/flare.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/brisk_2d.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/cppf.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/cvfh.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/our_cvfh.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/crh.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/don.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/feature.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/fpfh.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/fpfh_omp.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/from_meshes.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/gasd.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/gfpfh.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/integral_image2D.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/integral_image_normal.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/intensity_gradient.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/intensity_spin.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/linear_least_squares_normal.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/moment_invariants.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/moment_of_inertia_estimation.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/multiscale_feature_persistence.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/narf.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/narf_descriptor.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/normal_3d.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/normal_3d_omp.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/normal_based_signature.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/organized_edge_detection.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/pfh.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/pfh_tools.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/pfhrgb.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/ppf.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/ppfrgb.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/shot.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/shot_lrf.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/shot_lrf_omp.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/shot_omp.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/spin_image.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/principal_curvatures.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/rift.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/rops_estimation.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/rsd.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/grsd.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/statistical_multiscale_interest_region_extraction.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/vfh.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/esf.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/3dsc.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/usc.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/boundary.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/range_image_border_extractor.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_features" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/features/impl" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/board.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/flare.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/brisk_2d.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/cppf.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/cvfh.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/our_cvfh.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/crh.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/don.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/feature.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/fpfh.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/fpfh_omp.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/gasd.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/gfpfh.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/integral_image2D.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/integral_image_normal.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/intensity_gradient.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/intensity_spin.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/linear_least_squares_normal.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/moment_invariants.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/moment_of_inertia_estimation.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/multiscale_feature_persistence.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/narf.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/normal_3d.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/normal_3d_omp.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/normal_based_signature.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/organized_edge_detection.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/pfh.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/pfhrgb.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/ppf.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/ppfrgb.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/shot.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/shot_lrf.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/shot_lrf_omp.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/shot_omp.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/spin_image.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/principal_curvatures.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/rift.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/rops_estimation.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/rsd.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/grsd.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/statistical_multiscale_interest_region_extraction.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/vfh.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/esf.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/3dsc.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/usc.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/boundary.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/features/include/pcl/features/impl/range_image_border_extractor.hpp"
    )
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/rainple/nav_test/third_party/dddmr_build/pcl15/features/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
