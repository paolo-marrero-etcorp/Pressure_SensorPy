# Install script for directory: /home/src/Anjay/avs_commons/git

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avs_commons/avs_commons-targets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avs_commons/avs_commons-targets.cmake"
         "/home/src/Anjay/avs_commons/git/CMakeFiles/Export/lib/avs_commons/avs_commons-targets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avs_commons/avs_commons-targets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avs_commons/avs_commons-targets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/avs_commons" TYPE FILE FILES "/home/src/Anjay/avs_commons/git/CMakeFiles/Export/lib/avs_commons/avs_commons-targets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/avs_commons" TYPE FILE FILES "/home/src/Anjay/avs_commons/git/CMakeFiles/Export/lib/avs_commons/avs_commons-targets-noconfig.cmake")
  endif()
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/avs_commons" TYPE FILE FILES
    "/home/src/Anjay/avs_commons/git/avs_commons-config.cmake"
    "/home/src/Anjay/avs_commons/git/avs_commons-version.cmake"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/avs_commons/cmake" TYPE FILE FILES
    "/home/src/Anjay/avs_commons/git/cmake/FindMbedTLS.cmake"
    "/home/src/Anjay/avs_commons/git/cmake/FindTinyDTLS.cmake"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/src/Anjay/avs_commons/git/include_public/" FILES_MATCHING REGEX "[.]h$")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/src/Anjay/avs_commons/git/include_public/" FILES_MATCHING REGEX "[.]h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/src/Anjay/avs_commons/git/algorithm/cmake_install.cmake")
  include("/home/src/Anjay/avs_commons/git/buffer/cmake_install.cmake")
  include("/home/src/Anjay/avs_commons/git/list/cmake_install.cmake")
  include("/home/src/Anjay/avs_commons/git/vector/cmake_install.cmake")
  include("/home/src/Anjay/avs_commons/git/utils/cmake_install.cmake")
  include("/home/src/Anjay/avs_commons/git/net/cmake_install.cmake")
  include("/home/src/Anjay/avs_commons/git/stream/cmake_install.cmake")
  include("/home/src/Anjay/avs_commons/git/log/cmake_install.cmake")
  include("/home/src/Anjay/avs_commons/git/rbtree/cmake_install.cmake")
  include("/home/src/Anjay/avs_commons/git/coap/cmake_install.cmake")
  include("/home/src/Anjay/avs_commons/git/persistence/cmake_install.cmake")
  include("/home/src/Anjay/avs_commons/git/compat/threading/cmake_install.cmake")

endif()

