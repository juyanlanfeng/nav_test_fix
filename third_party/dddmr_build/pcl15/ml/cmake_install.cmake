# Install script for directory: /home/rainple/nav_test/third_party/dddmr_sources/pcl/ml

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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_ml" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_ml.so.1.15.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_ml.so.1.15"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/home/rainple/nav_test/third_party/dddmr_pcl15/lib")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_ml.so.1.15.0"
    "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_ml.so.1.15"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_ml.so.1.15.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_ml.so.1.15"
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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_ml" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/rainple/nav_test/third_party/dddmr_build/pcl15/lib/libpcl_ml.so")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_ml" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/rainple/nav_test/third_party/dddmr_build/pcl15/ml/pcl_ml.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_ml" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/ml" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/branch_estimator.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/feature_handler.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/multi_channel_2d_comparison_feature.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/multi_channel_2d_comparison_feature_handler.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/multi_channel_2d_data_set.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/multiple_data_2d_example_index.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/point_xy_32i.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/point_xy_32f.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/regression_variance_stats_estimator.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/stats_estimator.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/densecrf.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/pairwise_potential.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/permutohedral.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/svm_wrapper.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/svm.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/kmeans.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_ml" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/ml/dt" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/dt/decision_forest.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/dt/decision_forest_evaluator.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/dt/decision_forest_trainer.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/dt/decision_tree.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/dt/decision_tree_evaluator.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/dt/decision_tree_trainer.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/dt/decision_tree_data_provider.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_ml" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/ml/ferns" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/ferns/fern.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/ferns/fern_evaluator.h"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/ferns/fern_trainer.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_ml" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/ml/impl/dt" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/impl/dt/decision_forest_evaluator.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/impl/dt/decision_forest_trainer.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/impl/dt/decision_tree_evaluator.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/impl/dt/decision_tree_trainer.hpp"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_ml" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/ml/impl/ferns" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/impl/ferns/fern_evaluator.hpp"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/impl/ferns/fern_trainer.hpp"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_ml" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl/ml/impl/svm" TYPE FILE FILES "/home/rainple/nav_test/third_party/dddmr_sources/pcl/ml/include/pcl/ml/impl/svm/svm_wrapper.hpp")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/rainple/nav_test/third_party/dddmr_build/pcl15/ml/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
