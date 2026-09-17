# Install script for directory: /home/rainple/nav_test/third_party/dddmr_sources/pcl/common

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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_common" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_common.so.1.15.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_common.so.1.15"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/home/rainple/nav_test/third_party/dddmr_pcl15/lib")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_common.so.1.15.0"
    "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_common.so.1.15"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_common.so.1.15.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_common.so.1.15"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "::::::::::::::::::::::::::::::::::::::::::::::::::"
           NEW_RPATH "/home/rainple/nav_test/third_party/dddmr_pcl15/lib")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_common" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_common.so")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_common" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/rainple/nav_test/third_party/dddmr_build/pcl15/common/pcl_common.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_common" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/correspondence.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/memory.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/exceptions.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/pcl_base.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/pcl_exports.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/pcl_macros.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/types.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/point_cloud.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/point_struct_traits.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/type_traits.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/point_types_conversion.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/point_representation.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/point_types.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/for_each_type.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/pcl_tests.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/cloud_iterator.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/TextureMesh.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/sse.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/PCLPointField.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/PCLPointCloud2.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/PCLImage.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/PCLHeader.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/ModelCoefficients.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/PolygonMesh.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/Vertices.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/PointIndices.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/register_point_struct.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/conversions.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_common" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/common" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/angles.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/bivariate_polynomial.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/centroid.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/concatenate.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/common.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/common_headers.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/distances.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/eigen.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/copy_point.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/io.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/file_io.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/intersections.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/norms.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/pcl_filesystem.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/piecewise_linear_function.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/polynomial_calculations.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/poses_from_matches.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/time.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/time_trigger.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/transforms.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/transformation_from_correspondences.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/vector_average.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/pca.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/point_tests.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/synchronizer.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/utils.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/geometry.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/gaussian.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/spring.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/intensity.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/random.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/generate.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/projection_matrix.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/colors.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/feature_histogram.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_common" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/common/fft" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/fft/_kiss_fft_guts.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/fft/kiss_fft.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/fft/kiss_fftr.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_common" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/common/impl" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/impl/angles.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/impl/bivariate_polynomial.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/impl/centroid.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/impl/common.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/impl/eigen.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/impl/intersections.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/impl/copy_point.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/impl/io.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/impl/file_io.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/impl/norms.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/impl/piecewise_linear_function.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/impl/polynomial_calculations.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/impl/pca.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/impl/transforms.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/impl/transformation_from_correspondences.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/impl/vector_average.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/impl/gaussian.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/impl/spring.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/impl/intensity.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/impl/random.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/impl/generate.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/impl/projection_matrix.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/common/impl/accumulators.hpp"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_common" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/impl" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/impl/pcl_base.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/impl/instantiate.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/impl/point_types.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/impl/cloud_iterator.hpp"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_common" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/console" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/console/parse.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/console/print.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/console/time.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_common" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/range_image" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/range_image/bearing_angle_image.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/range_image/range_image.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/range_image/range_image_planar.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/range_image/range_image_spherical.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_common" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/range_image/impl" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/range_image/impl/range_image.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/range_image/impl/range_image_planar.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/common/include/pcl/range_image/impl/range_image_spherical.hpp"
    )
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/rainple/nav_test/third_party/dddmr_build/pcl15/common/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
