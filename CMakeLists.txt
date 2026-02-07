cmake_minimum_required(VERSION 3.20)
project(godot_mujoco_c_bridge C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_library(godot_mujoco_bridge SHARED
  src/gmj_bridge.c
)

target_include_directories(godot_mujoco_bridge
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h)
find_library(MUJOCO_LIBRARY mujoco)

if(MUJOCO_INCLUDE_DIR)
  target_include_directories(godot_mujoco_bridge PRIVATE ${MUJOCO_INCLUDE_DIR})
endif()

if(MUJOCO_LIBRARY)
  target_link_libraries(godot_mujoco_bridge PRIVATE ${MUJOCO_LIBRARY})
endif()

if(APPLE)
  set_target_properties(godot_mujoco_bridge PROPERTIES SUFFIX ".dylib")
elseif(UNIX)
  set_target_properties(godot_mujoco_bridge PROPERTIES SUFFIX ".so")
elseif(WIN32)
  set_target_properties(godot_mujoco_bridge PROPERTIES SUFFIX ".dll")
endif()
