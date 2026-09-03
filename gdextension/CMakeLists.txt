cmake_minimum_required(VERSION 3.20)
project(godot_mujoco_gdextension CXX)

include(FetchContent)

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
set(MUJOCO_VERSION "3.4.0" CACHE STRING "MuJoCo release version to fetch")
set(GODOT_CPP_TAG "godot-4.5-stable" CACHE STRING "godot-cpp git tag to build against")

# Where the built extension + MuJoCo runtime are staged so the demo project
# (and any exported game) can load them with zero manual steps.
set(GMJ_ADDON_BIN "${CMAKE_CURRENT_SOURCE_DIR}/demo/addons/godot_mujoco/bin"
    CACHE PATH "Directory that receives the extension and bundled MuJoCo runtime")

# ---------------------------------------------------------------------------
# Auto-fetch the prebuilt MuJoCo runtime (no manual install, no extra steps)
# ---------------------------------------------------------------------------
set(_mj_base "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}")

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(_mj_asset "mujoco-${MUJOCO_VERSION}-linux-aarch64.tar.gz")
  else()
    set(_mj_asset "mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz")
  endif()
  # Pin the linux-x86_64 archive for supply-chain integrity.
  if(MUJOCO_VERSION STREQUAL "3.4.0" AND _mj_asset MATCHES "linux-x86_64")
    set(_mj_hash "SHA256=3b0e47d8880da0005b61fe5e43a4d058eb5097e6e3ba44f4a23ce2b5fab2244d")
  endif()
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
  set(_mj_asset "mujoco-${MUJOCO_VERSION}-windows-x86_64.zip")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
  # The macOS release ships as a .dmg which CMake cannot extract; see README
  # for the documented macOS bring-up path.
  message(WARNING "Automatic MuJoCo fetch is not wired for macOS .dmg; see gdextension/README.md")
  set(_mj_asset "")
endif()

if(_mj_asset STREQUAL "")
  message(FATAL_ERROR "No MuJoCo asset mapping for this platform (${CMAKE_SYSTEM_NAME}/${CMAKE_SYSTEM_PROCESSOR}).")
endif()

if(DEFINED _mj_hash)
  FetchContent_Declare(mujoco URL "${_mj_base}/${_mj_asset}" URL_HASH "${_mj_hash}"
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
else()
  FetchContent_Declare(mujoco URL "${_mj_base}/${_mj_asset}" DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
endif()

FetchContent_MakeAvailable(mujoco)

set(MUJOCO_INCLUDE_DIR "${mujoco_SOURCE_DIR}/include")
if(WIN32)
  find_library(MUJOCO_LINK_LIB mujoco PATHS "${mujoco_SOURCE_DIR}/lib" NO_DEFAULT_PATH)
else()
  find_library(MUJOCO_LINK_LIB mujoco PATHS "${mujoco_SOURCE_DIR}/lib" NO_DEFAULT_PATH)
endif()

if(NOT MUJOCO_LINK_LIB)
  message(FATAL_ERROR "Could not locate MuJoCo library under ${mujoco_SOURCE_DIR}/lib")
endif()
message(STATUS "MuJoCo include: ${MUJOCO_INCLUDE_DIR}")
message(STATUS "MuJoCo library: ${MUJOCO_LINK_LIB}")

# ---------------------------------------------------------------------------
# Auto-fetch godot-cpp and build the C++ bindings
# ---------------------------------------------------------------------------
FetchContent_Declare(godot-cpp
    GIT_REPOSITORY https://github.com/godotengine/godot-cpp.git
    GIT_TAG ${GODOT_CPP_TAG}
    GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(godot-cpp)

# ---------------------------------------------------------------------------
# The GDExtension itself
# ---------------------------------------------------------------------------
add_library(godot_mujoco SHARED
    src/register_types.cpp
    src/mj_world.cpp)

target_link_libraries(godot_mujoco PRIVATE godot-cpp "${MUJOCO_LINK_LIB}")
target_include_directories(godot_mujoco PRIVATE "${MUJOCO_INCLUDE_DIR}")

get_target_property(GODOTCPP_SUFFIX godot-cpp GODOTCPP_SUFFIX)

set_target_properties(godot_mujoco PROPERTIES
    CXX_STANDARD 17
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
    PREFIX "lib"
    OUTPUT_NAME "godot_mujoco${GODOTCPP_SUFFIX}"
    # Stage with only the relocatable RPATH ($ORIGIN/@loader_path), never the
    # absolute build path, so the bundled runtime is fully portable.
    BUILD_WITH_INSTALL_RPATH TRUE)

# Resolve the bundled MuJoCo runtime next to the extension so no LD_LIBRARY_PATH
# / DYLD_LIBRARY_PATH / PATH tweaks are ever required.
if(APPLE)
  set_target_properties(godot_mujoco PROPERTIES
      BUILD_RPATH "@loader_path"
      INSTALL_RPATH "@loader_path")
elseif(UNIX)
  set_target_properties(godot_mujoco PROPERTIES
      BUILD_RPATH "$ORIGIN"
      INSTALL_RPATH "$ORIGIN")
endif()

# ---------------------------------------------------------------------------
# Stage the extension + MuJoCo runtime into the demo addon bin folder
# ---------------------------------------------------------------------------
file(GLOB MUJOCO_RUNTIME_FILES
    "${mujoco_SOURCE_DIR}/lib/*.so*"
    "${mujoco_SOURCE_DIR}/lib/*.dylib"
    "${mujoco_SOURCE_DIR}/bin/*.dll"
    "${mujoco_SOURCE_DIR}/lib/*.dll")

add_custom_command(TARGET godot_mujoco POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${GMJ_ADDON_BIN}"
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:godot_mujoco>" "${GMJ_ADDON_BIN}/"
    COMMENT "Staging godot_mujoco extension into ${GMJ_ADDON_BIN}")

foreach(_rt IN LISTS MUJOCO_RUNTIME_FILES)
  add_custom_command(TARGET godot_mujoco POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy "${_rt}" "${GMJ_ADDON_BIN}/"
      COMMENT "Bundling MuJoCo runtime: ${_rt}")
endforeach()
