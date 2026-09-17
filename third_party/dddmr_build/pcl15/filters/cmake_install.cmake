# Install script for directory: /home/rainple/nav_test/third_party/dddmr_sources/pcl/filters

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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_filters" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_filters.so.1.15.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_filters.so.1.15"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/home/rainple/nav_test/third_party/dddmr_pcl15/lib")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_filters.so.1.15.0"
    "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_filters.so.1.15"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_filters.so.1.15.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_filters.so.1.15"
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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_filters" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_filters.so")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_filters" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/rainple/nav_test/third_party/dddmr_build/pcl15/filters/pcl_filters.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_filters" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/filters" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/conditional_removal.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/crop_box.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/clipper3D.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/plane_clipper3D.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/box_clipper3D.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/crop_hull.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/extract_indices.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/filter.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/filter_indices.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/passthrough.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/shadowpoints.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/project_inliers.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/radius_outlier_removal.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/random_sample.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/normal_space.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/sampling_surface_normal.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/statistical_outlier_removal.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/voxel_grid.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/approximate_voxel_grid.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/bilateral.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/fast_bilateral.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/fast_bilateral_omp.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/voxel_grid_covariance.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/convolution.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/convolution_3d.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/voxel_grid_label.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/voxel_grid_occlusion_estimation.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/frustum_culling.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/covariance_sampling.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/median_filter.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/uniform_sampling.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/normal_refinement.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/grid_minimum.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/morphological_filter.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/local_maximum.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/model_outlier_removal.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/farthest_point_sampling.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_filters" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/filters/experimental" TYPE FILE FILES "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/experimental/functor_filter.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_filters" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/filters/impl" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/conditional_removal.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/crop_box.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/crop_hull.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/plane_clipper3D.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/box_clipper3D.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/extract_indices.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/filter.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/filter_indices.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/passthrough.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/shadowpoints.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/project_inliers.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/radius_outlier_removal.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/random_sample.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/normal_space.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/sampling_surface_normal.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/statistical_outlier_removal.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/voxel_grid.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/approximate_voxel_grid.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/bilateral.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/fast_bilateral.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/fast_bilateral_omp.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/voxel_grid_covariance.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/convolution.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/convolution_3d.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/voxel_grid_occlusion_estimation.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/frustum_culling.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/covariance_sampling.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/median_filter.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/uniform_sampling.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/normal_refinement.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/grid_minimum.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/morphological_filter.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/local_maximum.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/model_outlier_removal.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/filters/include/pcl/filters/impl/farthest_point_sampling.hpp"
    )
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/rainple/nav_test/third_party/dddmr_build/pcl15/filters/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
