# Install script for directory: /home/rainple/nav_test/third_party/dddmr_sources/pcl/surface

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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_surface" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_surface.so.1.15.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_surface.so.1.15"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/home/rainple/nav_test/third_party/dddmr_pcl15/lib")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_surface.so.1.15.0"
    "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_surface.so.1.15"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_surface.so.1.15.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_surface.so.1.15"
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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_surface" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_surface.so")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_surface" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/rainple/nav_test/third_party/dddmr_build/pcl15/surface/pcl_surface.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_surface" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/surface" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/ear_clipping.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/gp3.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/grid_projection.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/marching_cubes.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/marching_cubes_hoppe.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/marching_cubes_rbf.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/bilateral_upsampling.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/mls.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/organized_fast_mesh.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/reconstruction.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/processing.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/simplification_remove_unused_vertices.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/surfel_smoothing.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/texture_mapping.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/poisson.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/concave_hull.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/convex_hull.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/qhull.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_surface" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/surface/3rdparty/poisson4" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/3rdparty/poisson4/allocator.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/3rdparty/poisson4/binary_node.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/3rdparty/poisson4/bspline_data.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/3rdparty/poisson4/factor.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/3rdparty/poisson4/function_data.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/3rdparty/poisson4/geometry.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/3rdparty/poisson4/marching_cubes_poisson.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/3rdparty/poisson4/mat.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/3rdparty/poisson4/multi_grid_octree_data.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/3rdparty/poisson4/octree_poisson.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/3rdparty/poisson4/polynomial.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/3rdparty/poisson4/ppolynomial.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/3rdparty/poisson4/sparse_matrix.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/3rdparty/poisson4/vector.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/3rdparty/poisson4/bspline_data.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/3rdparty/poisson4/function_data.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/3rdparty/poisson4/geometry.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/3rdparty/poisson4/mat.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/3rdparty/poisson4/multi_grid_octree_data.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/3rdparty/poisson4/octree_poisson.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/3rdparty/poisson4/polynomial.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/3rdparty/poisson4/ppolynomial.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/3rdparty/poisson4/sparse_matrix.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/3rdparty/poisson4/vector.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/3rdparty/poisson4/poisson_exceptions.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_surface" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/surface/impl" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/impl/gp3.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/impl/grid_projection.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/impl/marching_cubes.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/impl/marching_cubes_hoppe.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/impl/marching_cubes_rbf.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/impl/bilateral_upsampling.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/impl/mls.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/impl/organized_fast_mesh.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/impl/reconstruction.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/impl/processing.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/impl/surfel_smoothing.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/impl/texture_mapping.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/impl/poisson.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/impl/concave_hull.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/surface/include/pcl/surface/impl/convex_hull.hpp"
    )
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/rainple/nav_test/third_party/dddmr_build/pcl15/surface/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
