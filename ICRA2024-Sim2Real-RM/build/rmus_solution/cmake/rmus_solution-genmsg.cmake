# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rmus_solution: 0 messages, 3 services")

set(MSG_I_FLAGS "-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rmus_solution_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/setgoal.srv" NAME_WE)
add_custom_target(_rmus_solution_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rmus_solution" "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/setgoal.srv" ""
)

get_filename_component(_filename "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/switch.srv" NAME_WE)
add_custom_target(_rmus_solution_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rmus_solution" "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/switch.srv" ""
)

get_filename_component(_filename "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/graspsignal.srv" NAME_WE)
add_custom_target(_rmus_solution_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rmus_solution" "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/graspsignal.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(rmus_solution
  "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/setgoal.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rmus_solution
)
_generate_srv_cpp(rmus_solution
  "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/switch.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rmus_solution
)
_generate_srv_cpp(rmus_solution
  "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/graspsignal.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rmus_solution
)

### Generating Module File
_generate_module_cpp(rmus_solution
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rmus_solution
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rmus_solution_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rmus_solution_generate_messages rmus_solution_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/setgoal.srv" NAME_WE)
add_dependencies(rmus_solution_generate_messages_cpp _rmus_solution_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/switch.srv" NAME_WE)
add_dependencies(rmus_solution_generate_messages_cpp _rmus_solution_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/graspsignal.srv" NAME_WE)
add_dependencies(rmus_solution_generate_messages_cpp _rmus_solution_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rmus_solution_gencpp)
add_dependencies(rmus_solution_gencpp rmus_solution_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rmus_solution_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(rmus_solution
  "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/setgoal.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rmus_solution
)
_generate_srv_eus(rmus_solution
  "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/switch.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rmus_solution
)
_generate_srv_eus(rmus_solution
  "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/graspsignal.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rmus_solution
)

### Generating Module File
_generate_module_eus(rmus_solution
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rmus_solution
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(rmus_solution_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(rmus_solution_generate_messages rmus_solution_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/setgoal.srv" NAME_WE)
add_dependencies(rmus_solution_generate_messages_eus _rmus_solution_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/switch.srv" NAME_WE)
add_dependencies(rmus_solution_generate_messages_eus _rmus_solution_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/graspsignal.srv" NAME_WE)
add_dependencies(rmus_solution_generate_messages_eus _rmus_solution_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rmus_solution_geneus)
add_dependencies(rmus_solution_geneus rmus_solution_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rmus_solution_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(rmus_solution
  "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/setgoal.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rmus_solution
)
_generate_srv_lisp(rmus_solution
  "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/switch.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rmus_solution
)
_generate_srv_lisp(rmus_solution
  "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/graspsignal.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rmus_solution
)

### Generating Module File
_generate_module_lisp(rmus_solution
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rmus_solution
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rmus_solution_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rmus_solution_generate_messages rmus_solution_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/setgoal.srv" NAME_WE)
add_dependencies(rmus_solution_generate_messages_lisp _rmus_solution_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/switch.srv" NAME_WE)
add_dependencies(rmus_solution_generate_messages_lisp _rmus_solution_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/graspsignal.srv" NAME_WE)
add_dependencies(rmus_solution_generate_messages_lisp _rmus_solution_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rmus_solution_genlisp)
add_dependencies(rmus_solution_genlisp rmus_solution_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rmus_solution_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(rmus_solution
  "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/setgoal.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rmus_solution
)
_generate_srv_nodejs(rmus_solution
  "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/switch.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rmus_solution
)
_generate_srv_nodejs(rmus_solution
  "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/graspsignal.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rmus_solution
)

### Generating Module File
_generate_module_nodejs(rmus_solution
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rmus_solution
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(rmus_solution_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(rmus_solution_generate_messages rmus_solution_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/setgoal.srv" NAME_WE)
add_dependencies(rmus_solution_generate_messages_nodejs _rmus_solution_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/switch.srv" NAME_WE)
add_dependencies(rmus_solution_generate_messages_nodejs _rmus_solution_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/graspsignal.srv" NAME_WE)
add_dependencies(rmus_solution_generate_messages_nodejs _rmus_solution_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rmus_solution_gennodejs)
add_dependencies(rmus_solution_gennodejs rmus_solution_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rmus_solution_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(rmus_solution
  "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/setgoal.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rmus_solution
)
_generate_srv_py(rmus_solution
  "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/switch.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rmus_solution
)
_generate_srv_py(rmus_solution
  "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/graspsignal.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rmus_solution
)

### Generating Module File
_generate_module_py(rmus_solution
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rmus_solution
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rmus_solution_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rmus_solution_generate_messages rmus_solution_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/setgoal.srv" NAME_WE)
add_dependencies(rmus_solution_generate_messages_py _rmus_solution_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/switch.srv" NAME_WE)
add_dependencies(rmus_solution_generate_messages_py _rmus_solution_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/meta_sim2024_ep_docker/ICRA2024-Sim2Real-RM/src/rmus_solution/srv/graspsignal.srv" NAME_WE)
add_dependencies(rmus_solution_generate_messages_py _rmus_solution_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rmus_solution_genpy)
add_dependencies(rmus_solution_genpy rmus_solution_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rmus_solution_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rmus_solution)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rmus_solution
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(rmus_solution_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(rmus_solution_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rmus_solution)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rmus_solution
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(rmus_solution_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(rmus_solution_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rmus_solution)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rmus_solution
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(rmus_solution_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(rmus_solution_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rmus_solution)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rmus_solution
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(rmus_solution_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(rmus_solution_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rmus_solution)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rmus_solution\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rmus_solution
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(rmus_solution_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(rmus_solution_generate_messages_py std_msgs_generate_messages_py)
endif()
