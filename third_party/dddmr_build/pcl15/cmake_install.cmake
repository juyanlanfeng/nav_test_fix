# Install script for directory: /home/rainple/nav_test/third_party/dddmr_sources/pcl

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.15/pcl" TYPE FILE FILES "/home/rainple/nav_test/third_party/dddmr_build/pcl15/include/pcl/pcl_config.h")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/common/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/kdtree/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/octree/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/search/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/sample_consensus/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/filters/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/2d/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/geometry/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/io/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/features/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/ml/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/segmentation/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/visualization/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/surface/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/registration/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/keypoints/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/tracking/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/recognition/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/stereo/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/apps/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/benchmarks/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/cuda/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/outofcore/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/examples/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/gpu/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/people/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/simulation/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/test/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/tools/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/rainple/nav_test/third_party/dddmr_build/pcl15/doc/cmake_install.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pclconfig" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pcl-1.15/Modules" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/cmake/Modules/FindClangFormat.cmake"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/cmake/Modules/FindDSSDK.cmake"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/cmake/Modules/FindEnsenso.cmake"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/cmake/Modules/FindFLANN.cmake"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/cmake/Modules/FindOpenNI.cmake"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/cmake/Modules/FindOpenNI2.cmake"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/cmake/Modules/FindPcap.cmake"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/cmake/Modules/FindQhull.cmake"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/cmake/Modules/FindRSSDK.cmake"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/cmake/Modules/FindRSSDK2.cmake"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/cmake/Modules/FindSphinx.cmake"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/cmake/Modules/FinddavidSDK.cmake"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/cmake/Modules/Findlibusb.cmake"
    "/home/rainple/nav_test/third_party/dddmr_sources/pcl/cmake/Modules/UseCompilerCache.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pclconfig" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pcl-1.15" TYPE FILE FILES
    "/home/rainple/nav_test/third_party/dddmr_build/pcl15/PCLConfig.cmake"
    "/home/rainple/nav_test/third_party/dddmr_build/pcl15/PCLConfigVersion.cmake"
    )
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/rainple/nav_test/third_party/dddmr_build/pcl15/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
if(CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_COMPONENT MATCHES "^[a-zA-Z0-9_.+-]+$")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
  else()
    string(MD5 CMAKE_INST_COMP_HASH "${CMAKE_INSTALL_COMPONENT}")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INST_COMP_HASH}.txt")
    unset(CMAKE_INST_COMP_HASH)
  endif()
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/rainple/nav_test/third_party/dddmr_build/pcl15/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
