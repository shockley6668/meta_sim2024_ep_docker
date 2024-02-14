# Install script for directory: /home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmus_solution/srv" TYPE FILE FILES
    "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/setgoal.srv"
    "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/switch.srv"
    "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/graspsignal.srv"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmus_solution/cmake" TYPE FILE FILES "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/build/rmus_solution/catkin_generated/installspace/rmus_solution-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/devel/include/rmus_solution")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/devel/share/roseus/ros/rmus_solution")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/devel/share/common-lisp/ros/rmus_solution")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/devel/share/gennodejs/ros/rmus_solution")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/devel/lib/python3/dist-packages/rmus_solution")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/devel/lib/python3/dist-packages/rmus_solution")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/build/rmus_solution/catkin_generated/installspace/rmus_solution.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmus_solution/cmake" TYPE FILE FILES "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/build/rmus_solution/catkin_generated/installspace/rmus_solution-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmus_solution/cmake" TYPE FILE FILES
    "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/build/rmus_solution/catkin_generated/installspace/rmus_solutionConfig.cmake"
    "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/build/rmus_solution/catkin_generated/installspace/rmus_solutionConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rmus_solution" TYPE FILE FILES "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/package.xml")
endif()

