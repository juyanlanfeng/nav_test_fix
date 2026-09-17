# Install script for directory: /home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition

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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_recognition" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_recognition.so.1.15.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_recognition.so.1.15"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/home/rainple/nav_test/third_party/dddmr_pcl15/lib")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_recognition.so.1.15.0"
    "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_recognition.so.1.15"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_recognition.so.1.15.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_recognition.so.1.15"
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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_recognition" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_recognition.so")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_recognition" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/rainple/nav_test/third_party/dddmr_build/pcl15/recognition/pcl_recognition.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_recognition" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/recognition" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/color_gradient_dot_modality.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/color_gradient_modality.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/color_modality.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/crh_alignment.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/linemod.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/dotmod.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/quantizable_modality.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/quantized_map.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/dot_modality.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/region_xy.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/mask_map.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/point_types.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/distance_map.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/dense_quantized_multi_mod_template.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/sparse_quantized_multi_mod_template.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/surface_normal_modality.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/implicit_shape_model.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_recognition" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/recognition/ransac_based" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/ransac_based/auxiliary.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/ransac_based/hypothesis.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/ransac_based/model_library.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/ransac_based/rigid_transform_space.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/ransac_based/obj_rec_ransac.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/ransac_based/orr_graph.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/ransac_based/orr_octree_zprojection.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/ransac_based/trimmed_icp.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/ransac_based/orr_octree.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/ransac_based/simple_octree.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/ransac_based/voxel_structure.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/ransac_based/bvh.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_recognition" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/recognition/hv" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/hv/occlusion_reasoning.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/hv/hypotheses_verification.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/hv/hv_papazov.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/hv/hv_go.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/hv/greedy_verification.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_recognition" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/recognition/cg" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/cg/correspondence_grouping.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/cg/hough_3d.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/cg/geometric_consistency.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_recognition" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/recognition/face_detection" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/face_detection/face_common.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/face_detection/face_detector_data_provider.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/face_detection/rf_face_detector_trainer.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/face_detection/rf_face_utils.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_recognition" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/recognition/impl" TYPE FILE FILES "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/impl/implicit_shape_model.hpp")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_recognition" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/recognition/impl/ransac_based" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/impl/ransac_based/simple_octree.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/impl/ransac_based/voxel_structure.hpp"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_recognition" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/recognition/impl/hv" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/impl/hv/occlusion_reasoning.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/impl/hv/hv_papazov.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/impl/hv/greedy_verification.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/impl/hv/hv_go.hpp"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_recognition" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/recognition/impl/cg" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/impl/cg/correspondence_grouping.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/impl/cg/hough_3d.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/impl/cg/geometric_consistency.hpp"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_recognition" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/recognition/linemod" TYPE FILE FILES "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/linemod/line_rgbd.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_recognition" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/recognition/impl/linemod" TYPE FILE FILES "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/impl/linemod/line_rgbd.hpp")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_recognition" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/recognition/3rdparty/metslib" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/3rdparty/metslib/abstract-search.hh"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/3rdparty/metslib/local-search.hh"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/3rdparty/metslib/mets.hh"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/3rdparty/metslib/metslib_config.hh"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/3rdparty/metslib/model.hh"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/3rdparty/metslib/observer.hh"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/3rdparty/metslib/simulated-annealing.hh"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/3rdparty/metslib/tabu-search.hh"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/recognition/include/pcl/recognition/3rdparty/metslib/termination-criteria.hh"
    )
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/rainple/nav_test/third_party/dddmr_build/pcl15/recognition/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
