# Install script for directory: /home/rainple/nav_test/third_party/dddmr_sources/pcl/registration

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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_registration" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_registration.so.1.15.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_registration.so.1.15"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/home/rainple/nav_test/third_party/dddmr_pcl15/lib")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_registration.so.1.15.0"
    "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_registration.so.1.15"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_registration.so.1.15.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_registration.so.1.15"
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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_registration" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_registration.so")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_registration" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/rainple/nav_test/third_party/dddmr_build/pcl15/registration/pcl_registration.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_registration" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/registration" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/boost_graph.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/convergence_criteria.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/default_convergence_criteria.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/correspondence_estimation.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/correspondence_estimation_normal_shooting.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/correspondence_estimation_backprojection.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/correspondence_estimation_organized_projection.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/correspondence_rejection.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/correspondence_rejection_distance.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/correspondence_rejection_median_distance.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/correspondence_rejection_surface_normal.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/correspondence_rejection_features.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/correspondence_rejection_one_to_one.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/correspondence_rejection_poly.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/correspondence_rejection_sample_consensus.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/correspondence_rejection_sample_consensus_2d.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/correspondence_rejection_trimmed.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/correspondence_rejection_var_trimmed.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/correspondence_rejection_organized_boundary.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/correspondence_sorting.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/correspondence_types.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/ia_ransac.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/icp.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/joint_icp.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/incremental_registration.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/icp_nl.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/lum.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/elch.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/meta_registration.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/ndt.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/ndt_2d.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/ppf_registration.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/pairwise_graph_registration.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/pyramid_feature_matching.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/registration.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/transformation_estimation.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/transformation_estimation_2D.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/transformation_estimation_svd.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/transformation_estimation_svd_scale.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/transformation_estimation_dual_quaternion.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/transformation_estimation_lm.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/transformation_estimation_point_to_plane.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/transformation_estimation_point_to_plane_weighted.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/transformation_estimation_point_to_plane_lls.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/transformation_estimation_point_to_plane_lls_weighted.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/transformation_estimation_symmetric_point_to_plane_lls.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/transformation_validation.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/transformation_validation_euclidean.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/gicp.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/gicp6d.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/bfgs.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/warp_point_rigid.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/warp_point_rigid_6d.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/warp_point_rigid_3d.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/distances.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/exceptions.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/sample_consensus_prerejective.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/ia_fpcs.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/ia_kfpcs.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/matching_candidate.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/transformation_estimation_3point.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_registration" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/registration/impl" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/default_convergence_criteria.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/correspondence_estimation.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/correspondence_estimation_normal_shooting.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/correspondence_estimation_backprojection.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/correspondence_estimation_organized_projection.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/correspondence_rejection_features.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/correspondence_rejection_poly.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/correspondence_rejection_sample_consensus.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/correspondence_rejection_sample_consensus_2d.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/correspondence_types.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/ia_ransac.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/icp.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/joint_icp.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/incremental_registration.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/icp_nl.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/elch.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/lum.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/meta_registration.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/ndt.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/ndt_2d.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/ppf_registration.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/pyramid_feature_matching.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/registration.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/transformation_estimation_2D.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/transformation_estimation_svd.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/transformation_estimation_svd_scale.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/transformation_estimation_dual_quaternion.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/transformation_estimation_lm.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/transformation_estimation_point_to_plane_lls.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/transformation_estimation_point_to_plane_lls_weighted.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/transformation_estimation_point_to_plane_weighted.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/transformation_estimation_symmetric_point_to_plane_lls.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/transformation_validation_euclidean.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/gicp.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/sample_consensus_prerejective.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/ia_fpcs.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/ia_kfpcs.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/registration/include/pcl/registration/impl/transformation_estimation_3point.hpp"
    )
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/rainple/nav_test/third_party/dddmr_build/pcl15/registration/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
