cmake_minimum_required(VERSION 3.20)
project(godot_mujoco_c_bridge C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(GMJ_COPY_TO_GODOT_PROJECTS "Copy built library into Godot project bin folders" ON)

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
  set_target_properties(godot_mujoco_bridge PROPERTIES
    BUILD_RPATH "@loader_path"
    INSTALL_RPATH "@loader_path"
  )
elseif(UNIX)
  set_target_properties(godot_mujoco_bridge PROPERTIES
    BUILD_RPATH "$ORIGIN"
    INSTALL_RPATH "$ORIGIN"
  )
endif()

if(APPLE)
  set_target_properties(godot_mujoco_bridge PROPERTIES SUFFIX ".dylib")
elseif(UNIX)
  set_target_properties(godot_mujoco_bridge PROPERTIES SUFFIX ".so")
elseif(WIN32)
  set_target_properties(godot_mujoco_bridge PROPERTIES SUFFIX ".dll")
endif()

if(GMJ_COPY_TO_GODOT_PROJECTS)
  set(GMJ_TARGET_BINS
    "${CMAKE_CURRENT_SOURCE_DIR}/godot_demo/bin"
    "${CMAKE_CURRENT_SOURCE_DIR}/example/bin"
  )

  foreach(GMJ_BIN_DIR IN LISTS GMJ_TARGET_BINS)
    add_custom_command(
      TARGET godot_mujoco_bridge
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E make_directory "${GMJ_BIN_DIR}"
      COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:godot_mujoco_bridge>" "${GMJ_BIN_DIR}/"
    )

    if(MUJOCO_LIBRARY)
      if(APPLE)
        get_filename_component(GMJ_MUJOCO_LIB_NAME "${MUJOCO_LIBRARY}" NAME)
        add_custom_command(
          TARGET godot_mujoco_bridge
          POST_BUILD
          COMMAND ${CMAKE_COMMAND} -E make_directory "${GMJ_BIN_DIR}/mujoco.framework/Versions/A"
          COMMAND ${CMAKE_COMMAND} -E copy "${MUJOCO_LIBRARY}" "${GMJ_BIN_DIR}/mujoco.framework/Versions/A/${GMJ_MUJOCO_LIB_NAME}"
        )
      elseif(UNIX)
        add_custom_command(
          TARGET godot_mujoco_bridge
          POST_BUILD
          COMMAND ${CMAKE_COMMAND} -E copy "${MUJOCO_LIBRARY}" "${GMJ_BIN_DIR}/"
        )
      endif()
    endif()
  endforeach()
endif()
