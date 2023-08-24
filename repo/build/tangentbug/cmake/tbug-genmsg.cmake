# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tbug: 7 messages, 0 services")

set(MSG_I_FLAGS "-Itbug:/home/z_lin/mapless/devel/share/tbug/msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tbug_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusFeedback.msg" NAME_WE)
add_custom_target(_tbug_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tbug" "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusFeedback.msg" ""
)

get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusGoal.msg" NAME_WE)
add_custom_target(_tbug_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tbug" "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusGoal.msg" ""
)

get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionResult.msg" NAME_WE)
add_custom_target(_tbug_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tbug" "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionResult.msg" "actionlib_msgs/GoalID:tbug/goalStatusResult:actionlib_msgs/GoalStatus:std_msgs/Header"
)

get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusAction.msg" NAME_WE)
add_custom_target(_tbug_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tbug" "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusAction.msg" "actionlib_msgs/GoalID:tbug/goalStatusResult:tbug/goalStatusGoal:tbug/goalStatusFeedback:tbug/goalStatusActionResult:tbug/goalStatusActionGoal:tbug/goalStatusActionFeedback:std_msgs/Header:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusResult.msg" NAME_WE)
add_custom_target(_tbug_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tbug" "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusResult.msg" ""
)

get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionGoal.msg" NAME_WE)
add_custom_target(_tbug_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tbug" "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionGoal.msg" "actionlib_msgs/GoalID:tbug/goalStatusGoal:std_msgs/Header"
)

get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionFeedback.msg" NAME_WE)
add_custom_target(_tbug_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tbug" "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionFeedback.msg" "actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:tbug/goalStatusFeedback:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tbug
)
_generate_msg_cpp(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tbug
)
_generate_msg_cpp(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tbug
)
_generate_msg_cpp(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusResult.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusGoal.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusFeedback.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionResult.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionGoal.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tbug
)
_generate_msg_cpp(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tbug
)
_generate_msg_cpp(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tbug
)
_generate_msg_cpp(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tbug
)

### Generating Services

### Generating Module File
_generate_module_cpp(tbug
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tbug
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tbug_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tbug_generate_messages tbug_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusFeedback.msg" NAME_WE)
add_dependencies(tbug_generate_messages_cpp _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusGoal.msg" NAME_WE)
add_dependencies(tbug_generate_messages_cpp _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionResult.msg" NAME_WE)
add_dependencies(tbug_generate_messages_cpp _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusAction.msg" NAME_WE)
add_dependencies(tbug_generate_messages_cpp _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusResult.msg" NAME_WE)
add_dependencies(tbug_generate_messages_cpp _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionGoal.msg" NAME_WE)
add_dependencies(tbug_generate_messages_cpp _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionFeedback.msg" NAME_WE)
add_dependencies(tbug_generate_messages_cpp _tbug_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tbug_gencpp)
add_dependencies(tbug_gencpp tbug_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tbug_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tbug
)
_generate_msg_eus(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tbug
)
_generate_msg_eus(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tbug
)
_generate_msg_eus(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusResult.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusGoal.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusFeedback.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionResult.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionGoal.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tbug
)
_generate_msg_eus(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tbug
)
_generate_msg_eus(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tbug
)
_generate_msg_eus(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tbug
)

### Generating Services

### Generating Module File
_generate_module_eus(tbug
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tbug
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tbug_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tbug_generate_messages tbug_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusFeedback.msg" NAME_WE)
add_dependencies(tbug_generate_messages_eus _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusGoal.msg" NAME_WE)
add_dependencies(tbug_generate_messages_eus _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionResult.msg" NAME_WE)
add_dependencies(tbug_generate_messages_eus _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusAction.msg" NAME_WE)
add_dependencies(tbug_generate_messages_eus _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusResult.msg" NAME_WE)
add_dependencies(tbug_generate_messages_eus _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionGoal.msg" NAME_WE)
add_dependencies(tbug_generate_messages_eus _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionFeedback.msg" NAME_WE)
add_dependencies(tbug_generate_messages_eus _tbug_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tbug_geneus)
add_dependencies(tbug_geneus tbug_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tbug_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tbug
)
_generate_msg_lisp(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tbug
)
_generate_msg_lisp(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tbug
)
_generate_msg_lisp(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusResult.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusGoal.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusFeedback.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionResult.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionGoal.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tbug
)
_generate_msg_lisp(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tbug
)
_generate_msg_lisp(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tbug
)
_generate_msg_lisp(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tbug
)

### Generating Services

### Generating Module File
_generate_module_lisp(tbug
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tbug
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tbug_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tbug_generate_messages tbug_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusFeedback.msg" NAME_WE)
add_dependencies(tbug_generate_messages_lisp _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusGoal.msg" NAME_WE)
add_dependencies(tbug_generate_messages_lisp _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionResult.msg" NAME_WE)
add_dependencies(tbug_generate_messages_lisp _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusAction.msg" NAME_WE)
add_dependencies(tbug_generate_messages_lisp _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusResult.msg" NAME_WE)
add_dependencies(tbug_generate_messages_lisp _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionGoal.msg" NAME_WE)
add_dependencies(tbug_generate_messages_lisp _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionFeedback.msg" NAME_WE)
add_dependencies(tbug_generate_messages_lisp _tbug_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tbug_genlisp)
add_dependencies(tbug_genlisp tbug_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tbug_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tbug
)
_generate_msg_nodejs(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tbug
)
_generate_msg_nodejs(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tbug
)
_generate_msg_nodejs(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusResult.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusGoal.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusFeedback.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionResult.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionGoal.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tbug
)
_generate_msg_nodejs(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tbug
)
_generate_msg_nodejs(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tbug
)
_generate_msg_nodejs(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tbug
)

### Generating Services

### Generating Module File
_generate_module_nodejs(tbug
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tbug
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tbug_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tbug_generate_messages tbug_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusFeedback.msg" NAME_WE)
add_dependencies(tbug_generate_messages_nodejs _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusGoal.msg" NAME_WE)
add_dependencies(tbug_generate_messages_nodejs _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionResult.msg" NAME_WE)
add_dependencies(tbug_generate_messages_nodejs _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusAction.msg" NAME_WE)
add_dependencies(tbug_generate_messages_nodejs _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusResult.msg" NAME_WE)
add_dependencies(tbug_generate_messages_nodejs _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionGoal.msg" NAME_WE)
add_dependencies(tbug_generate_messages_nodejs _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionFeedback.msg" NAME_WE)
add_dependencies(tbug_generate_messages_nodejs _tbug_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tbug_gennodejs)
add_dependencies(tbug_gennodejs tbug_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tbug_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tbug
)
_generate_msg_py(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tbug
)
_generate_msg_py(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tbug
)
_generate_msg_py(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusResult.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusGoal.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusFeedback.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionResult.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionGoal.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tbug
)
_generate_msg_py(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tbug
)
_generate_msg_py(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tbug
)
_generate_msg_py(tbug
  "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/z_lin/mapless/devel/share/tbug/msg/goalStatusFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tbug
)

### Generating Services

### Generating Module File
_generate_module_py(tbug
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tbug
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tbug_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tbug_generate_messages tbug_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusFeedback.msg" NAME_WE)
add_dependencies(tbug_generate_messages_py _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusGoal.msg" NAME_WE)
add_dependencies(tbug_generate_messages_py _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionResult.msg" NAME_WE)
add_dependencies(tbug_generate_messages_py _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusAction.msg" NAME_WE)
add_dependencies(tbug_generate_messages_py _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusResult.msg" NAME_WE)
add_dependencies(tbug_generate_messages_py _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionGoal.msg" NAME_WE)
add_dependencies(tbug_generate_messages_py _tbug_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/z_lin/mapless/devel/share/tbug/msg/goalStatusActionFeedback.msg" NAME_WE)
add_dependencies(tbug_generate_messages_py _tbug_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tbug_genpy)
add_dependencies(tbug_genpy tbug_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tbug_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tbug)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tbug
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(tbug_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(tbug_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tbug)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tbug
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(tbug_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(tbug_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tbug)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tbug
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(tbug_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(tbug_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tbug)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tbug
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(tbug_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(tbug_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tbug)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tbug\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tbug
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(tbug_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(tbug_generate_messages_py std_msgs_generate_messages_py)
endif()
