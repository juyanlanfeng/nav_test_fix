if(NOT EXISTS "/home/rainple/nav_test/third_party/dddmr_build/pcl15/install_manifest.txt")
  message(FATAL_ERROR "Cannot find install manifest: \"/home/rainple/nav_test/third_party/dddmr_build/pcl15/install_manifest.txt\"")
endif()

file(READ "/home/rainple/nav_test/third_party/dddmr_build/pcl15/install_manifest.txt" files)
string(REGEX REPLACE "\n" ";" files "${files}")
foreach(file ${files})
  message(STATUS "Uninstalling \"$ENV{DESTDIR}${file}\"")
  if(EXISTS "$ENV{DESTDIR}${file}" OR IS_SYMLINK "$ENV{DESTDIR}${file}")
    exec_program("/usr/local/bin/cmake" ARGS "-E remove \"$ENV{DESTDIR}${file}\""
                 OUTPUT_VARIABLE rm_out RETURN_VALUE rm_retval)
    if(NOT "${rm_retval}" STREQUAL 0)
      message(FATAL_ERROR "Problem when removing \"$ENV{DESTDIR}${file}\"")
    endif()
  else()
    message(STATUS "File \"$ENV{DESTDIR}${file}\" does not exist.")
  endif()
endforeach()

# remove pcl directory in include (removes all files in it!)
message(STATUS "Uninstalling \"/home/rainple/nav_test/third_party/dddmr_pcl15/include/pcl-1.15\"")
if(EXISTS "/home/rainple/nav_test/third_party/dddmr_pcl15/include/pcl-1.15")
  exec_program("/usr/local/bin/cmake"
               ARGS "-E remove_directory \"/home/rainple/nav_test/third_party/dddmr_pcl15/include/pcl-1.15\""
               OUTPUT_VARIABLE rm_out RETURN_VALUE rm_retval)
  if(NOT "${rm_retval}" STREQUAL 0)
    message(FATAL_ERROR "Problem when removing \"/home/rainple/nav_test/third_party/dddmr_pcl15/include/pcl-1.15\"")
  endif()
else()
  message(STATUS "Directory \"/home/rainple/nav_test/third_party/dddmr_pcl15/include/pcl-1.15\" does not exist.")
endif()

# remove pcl directory in share (removes all files in it!)
# created by CMakeLists.txt for PCLConfig.cmake
if(EXISTS "/home/rainple/nav_test/third_party/dddmr_pcl15/share/pcl-1.15")
  file(GLOB_RECURSE CMAKE_CONFIG_FOLDER_FILES FOLLOW_SYMLINKS
       LIST_DIRECTORIES false
       "/home/rainple/nav_test/third_party/dddmr_pcl15/share/pcl-1.15/*")
  list(LENGTH CMAKE_CONFIG_FOLDER_FILES CMAKE_CONFIG_FOLDER_FILES_NUMBER)
  if(CMAKE_CONFIG_FOLDER_FILES_NUMBER EQUAL 0)
    message(STATUS "Uninstalling \"/home/rainple/nav_test/third_party/dddmr_pcl15/share/pcl-1.15\"")
    exec_program("/usr/local/bin/cmake"
                 ARGS "-E remove_directory \"/home/rainple/nav_test/third_party/dddmr_pcl15/share/pcl-1.15\""
                 OUTPUT_VARIABLE rm_out RETURN_VALUE rm_retval)
    if(NOT "${rm_retval}" STREQUAL 0)
      message(FATAL_ERROR "Problem when removing \"/home/rainple/nav_test/third_party/dddmr_pcl15/share/pcl-1.15\"")
    endif()
  endif()
else()
  message(STATUS "Directory \"/home/rainple/nav_test/third_party/dddmr_pcl15/share/pcl-1.15\" does not exist.")
endif()

# remove pcl directory in share/doc (removes all files in it!)
if(OFF)
  message(STATUS "Uninstalling \"/home/rainple/nav_test/third_party/dddmr_pcl15/share/doc/pcl-1.15\"")
  if(EXISTS "/home/rainple/nav_test/third_party/dddmr_pcl15/share/doc/pcl-1.15")
    exec_program("/usr/local/bin/cmake"
                 ARGS "-E remove_directory \"/home/rainple/nav_test/third_party/dddmr_pcl15/share/doc/pcl-1.15\""
                 OUTPUT_VARIABLE rm_out RETURN_VALUE rm_retval)
    if(NOT "${rm_retval}" STREQUAL 0)
      message(FATAL_ERROR "Problem when removing \"/home/rainple/nav_test/third_party/dddmr_pcl15/share/doc/pcl-1.15\"")
    endif()
  else()
    message(STATUS "Directory \"/home/rainple/nav_test/third_party/dddmr_pcl15/share/doc/pcl-1.15\" does not exist.")
  endif()
endif()
