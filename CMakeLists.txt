cmake_minimum_required(VERSION 3.24)
project(godot_mujoco_gdextension LANGUAGES C CXX)

include(FetchContent)

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
set(MUJOCO_VERSION "3.12.0" CACHE STRING "MuJoCo release version to fetch")
set(GODOT_CPP_TAG "godot-4.5-stable" CACHE STRING "godot-cpp git tag to build against")

# Where the built extension + MuJoCo runtime are staged so the demo project
# (and any exported game) can load them with zero manual steps.
set(GMJ_ADDON_BIN "${CMAKE_CURRENT_SOURCE_DIR}/demo/addons/godot_mujoco/bin"
    CACHE PATH "Directory that receives the extension and bundled MuJoCo runtime")

# Desktop uses official prebuilts. Android / iOS have none, so those toolchains
# always compile the simulation core from source (no OpenGL / GLFW / simulate).
set(GMJ_MOBILE OFF)
if(ANDROID OR CMAKE_SYSTEM_NAME STREQUAL "iOS")
    set(GMJ_MOBILE ON)
endif()

option(GMJ_BUILD_MUJOCO_FROM_SOURCE
    "Build MuJoCo from source instead of official desktop prebuilts"
    ${GMJ_MOBILE})
if(GMJ_MOBILE)
    set(GMJ_BUILD_MUJOCO_FROM_SOURCE ON)
endif()

option(GMJ_ALLOW_UNVERIFIED "Allow fetching a MuJoCo archive that has no pinned checksum" OFF)
option(GMJ_BUILD_CORE_SMOKE
    "Build the standalone MuJoCo core smoke executable (auto-on for mobile)"
    ${GMJ_MOBILE})

# ---------------------------------------------------------------------------
# MuJoCo: from-source (mobile / opt-in) or official prebuilt (desktop)
# ---------------------------------------------------------------------------
set(GMJ_MUJOCO_FROM_SOURCE_TARGET FALSE)

if(GMJ_BUILD_MUJOCO_FROM_SOURCE)
    # Source tarball (not the desktop binary archive). Hash is for 3.12.0.
    set(_mj_src_url
        "https://github.com/google-deepmind/mujoco/archive/refs/tags/${MUJOCO_VERSION}.tar.gz")
    if(MUJOCO_VERSION STREQUAL "3.12.0")
        set(_mj_src_hash "SHA256=9faa979982c3e924e8aaff3b16983bba3a1ab19c81f4f73178ae9c2ea25e467d")
    endif()
    if(NOT DEFINED _mj_src_hash AND NOT GMJ_ALLOW_UNVERIFIED)
        message(FATAL_ERROR "No pinned checksum for MuJoCo source ${MUJOCO_VERSION}. "
            "Add its SHA-256 to CMakeLists.txt, or pass -DGMJ_ALLOW_UNVERIFIED=ON.")
    endif()

    # Simulation core only — no viewer, samples, tests, USD, or Filament.
    # Classic mjr_* still compiles (GLAD, runtime-loaded GL); we never call it.
    set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(MUJOCO_BUILD_SIMULATE OFF CACHE BOOL "" FORCE)
    set(MUJOCO_BUILD_STUDIO OFF CACHE BOOL "" FORCE)
    set(MUJOCO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(MUJOCO_TEST_PYTHON_UTIL OFF CACHE BOOL "" FORCE)
    set(MUJOCO_WITH_USD OFF CACHE BOOL "" FORCE)
    set(MUJOCO_USE_FILAMENT OFF CACHE BOOL "" FORCE)
    set(MUJOCO_BUILD_MACOS_FRAMEWORKS OFF CACHE BOOL "" FORCE)
    set(MUJOCO_ENABLE_AVX OFF CACHE BOOL "" FORCE)
    set(MUJOCO_ENABLE_AVX_INTRINSICS OFF CACHE BOOL "" FORCE)
    # LTO of the whole engine is slow and brittle on NDK/Xcode; desktop prebuilts
    # already ship optimized. Keep source builds incremental and CI-friendly.
    set(MUJOCO_ENABLE_LTO OFF CACHE BOOL "" FORCE)

    set(_mj_mobile_patch "${CMAKE_CURRENT_SOURCE_DIR}/cmake/patches/mujoco-mobile.patch")
    # posix_memalign + localtime_r: official sources assume desktop C11/POSIX.
    # apply_patch.cmake treats "already applied" as success so reconfigure is idempotent.
    set(_mj_patch_cmd ${CMAKE_COMMAND} -D PATCH_FILE=${_mj_mobile_patch}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/apply_patch.cmake")
    if(DEFINED _mj_src_hash)
        FetchContent_Declare(mujoco_src URL "${_mj_src_url}" URL_HASH "${_mj_src_hash}"
            DOWNLOAD_EXTRACT_TIMESTAMP TRUE
            PATCH_COMMAND ${_mj_patch_cmd})
    else()
        FetchContent_Declare(mujoco_src URL "${_mj_src_url}" DOWNLOAD_EXTRACT_TIMESTAMP TRUE
            PATCH_COMMAND ${_mj_patch_cmd})
    endif()
    FetchContent_MakeAvailable(mujoco_src)

    if(NOT TARGET mujoco)
        message(FATAL_ERROR "MuJoCo source build did not create a 'mujoco' target")
    endif()
    set(GMJ_MUJOCO_FROM_SOURCE_TARGET TRUE)
    set(MUJOCO_INCLUDE_DIR "${mujoco_src_SOURCE_DIR}/include")
    set(MUJOCO_LINK_LIB mujoco)
    message(STATUS "MuJoCo from source: ${mujoco_src_SOURCE_DIR}")
else()
    set(_mj_base "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}")

    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
            set(_mj_asset "mujoco-${MUJOCO_VERSION}-linux-aarch64.tar.gz")
        else()
            set(_mj_asset "mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz")
        endif()
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
        set(_mj_asset "mujoco-${MUJOCO_VERSION}-windows-x86_64.zip")
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        # The macOS release ships as a .dmg which CMake cannot extract. Point the
        # build at an already-extracted MuJoCo (the folder containing include/ and
        # lib/) via -DGMJ_MUJOCO_ROOT=/path/to/mujoco-<ver>; see README "Platforms".
        set(_mj_asset "")
    endif()

    # Pinned SHA-256 checksums per (version, asset) for supply-chain integrity.
    if(MUJOCO_VERSION STREQUAL "3.12.0")
        if(_mj_asset MATCHES "linux-x86_64")
            set(_mj_hash "SHA256=a9367911e6d5eaeade17c2197304687421c1fc932cdf7bcd4cb8cfaf0374dcb2")
        elseif(_mj_asset MATCHES "linux-aarch64")
            set(_mj_hash "SHA256=08fd5627a2ef7d5a42580c40e014ab2c1a644f082010c584ca361a3ed8cad838")
        elseif(_mj_asset MATCHES "windows-x86_64")
            set(_mj_hash "SHA256=ffe071c2747dd9513a1c59e7d2428bb678d887f9edec9eb9674b4288a248a8e9")
        endif()
    endif()

    # Allow pointing at a pre-extracted MuJoCo (required on macOS; also a general
    # offline / custom-version escape hatch).
    if(DEFINED GMJ_MUJOCO_ROOT AND NOT GMJ_MUJOCO_ROOT STREQUAL "")
        set(mujoco_SOURCE_DIR "${GMJ_MUJOCO_ROOT}")
    elseif(_mj_asset STREQUAL "")
        message(FATAL_ERROR "No MuJoCo archive is available for this platform "
            "(${CMAKE_SYSTEM_NAME}/${CMAKE_SYSTEM_PROCESSOR}). Pass "
            "-DGMJ_MUJOCO_ROOT=/path/to/extracted/mujoco or "
            "-DGMJ_BUILD_MUJOCO_FROM_SOURCE=ON.")
    else()
        if(NOT DEFINED _mj_hash AND NOT GMJ_ALLOW_UNVERIFIED)
            message(FATAL_ERROR "No pinned checksum for '${_mj_asset}'. Add its SHA-256 to CMakeLists.txt, "
                "or pass -DGMJ_ALLOW_UNVERIFIED=ON to fetch it unverified.")
        endif()
        if(DEFINED _mj_hash)
            FetchContent_Declare(mujoco URL "${_mj_base}/${_mj_asset}" URL_HASH "${_mj_hash}"
                DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
        else()
            FetchContent_Declare(mujoco URL "${_mj_base}/${_mj_asset}" DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
        endif()
        FetchContent_MakeAvailable(mujoco)
    endif()

    set(MUJOCO_INCLUDE_DIR "${mujoco_SOURCE_DIR}/include")
    find_library(MUJOCO_LINK_LIB mujoco PATHS "${mujoco_SOURCE_DIR}/lib" NO_DEFAULT_PATH)

    if(NOT MUJOCO_LINK_LIB)
        message(FATAL_ERROR "Could not locate MuJoCo library under ${mujoco_SOURCE_DIR}/lib")
    endif()
    message(STATUS "MuJoCo include: ${MUJOCO_INCLUDE_DIR}")
    message(STATUS "MuJoCo library: ${MUJOCO_LINK_LIB}")
endif()

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
# Standalone MuJoCo core smoke (no Godot). Used to prove the from-source
# simulation core on Android via adb, and as a link test on iOS.
# ---------------------------------------------------------------------------
if(GMJ_BUILD_CORE_SMOKE OR GMJ_MOBILE)
    add_executable(gmj_core_smoke src/mobile_core_smoke.cpp)
    target_link_libraries(gmj_core_smoke PRIVATE "${MUJOCO_LINK_LIB}")
    target_include_directories(gmj_core_smoke PRIVATE "${MUJOCO_INCLUDE_DIR}")
    set_target_properties(gmj_core_smoke PROPERTIES
        CXX_STANDARD 17
        CXX_EXTENSIONS OFF)
    if(ANDROID)
        # PIE executable that `adb push` + `adb shell` can run on a device/emulator.
        target_link_options(gmj_core_smoke PRIVATE -pie)
    endif()
endif()

# ---------------------------------------------------------------------------
# Stage the extension + MuJoCo runtime into the demo addon bin folder
# ---------------------------------------------------------------------------
add_custom_command(TARGET godot_mujoco POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${GMJ_ADDON_BIN}"
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:godot_mujoco>" "${GMJ_ADDON_BIN}/"
    COMMENT "Staging godot_mujoco extension into ${GMJ_ADDON_BIN}")

if(GMJ_MUJOCO_FROM_SOURCE_TARGET)
    add_custom_command(TARGET godot_mujoco POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:mujoco>"
            "${GMJ_ADDON_BIN}/$<TARGET_FILE_NAME:mujoco>"
        COMMENT "Bundling MuJoCo runtime from source")
    # Also drop the unversioned linker name so the .gdextension dependency path
    # stays stable across patch versions when the SONAME file is a symlink.
    add_custom_command(TARGET godot_mujoco POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:mujoco>"
            "${GMJ_ADDON_BIN}/$<TARGET_LINKER_FILE_NAME:mujoco>"
        COMMENT "Bundling MuJoCo linker-name copy")
    if(TARGET gmj_core_smoke)
        add_custom_command(TARGET gmj_core_smoke POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E make_directory "${GMJ_ADDON_BIN}"
            COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:gmj_core_smoke>"
                "${GMJ_ADDON_BIN}/"
            COMMENT "Staging gmj_core_smoke")
    endif()
    if(ANDROID AND DEFINED CMAKE_ANDROID_NDK)
        # Needed by adb-run of gmj_core_smoke; Godot's Android export also ships it.
        set(_gmj_libcxx "${CMAKE_ANDROID_NDK}/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/lib/aarch64-linux-android/libc++_shared.so")
        if(CMAKE_HOST_SYSTEM_NAME STREQUAL "Darwin")
            set(_gmj_libcxx "${CMAKE_ANDROID_NDK}/toolchains/llvm/prebuilt/darwin-x86_64/sysroot/usr/lib/aarch64-linux-android/libc++_shared.so")
            if(NOT EXISTS "${_gmj_libcxx}")
                set(_gmj_libcxx "${CMAKE_ANDROID_NDK}/toolchains/llvm/prebuilt/darwin-aarch64/sysroot/usr/lib/aarch64-linux-android/libc++_shared.so")
            endif()
        endif()
        if(EXISTS "${_gmj_libcxx}")
            add_custom_command(TARGET godot_mujoco POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy "${_gmj_libcxx}" "${GMJ_ADDON_BIN}/libc++_shared.so"
                COMMENT "Bundling NDK libc++_shared.so")
        endif()
    endif()
else()
    file(GLOB MUJOCO_RUNTIME_FILES
        "${mujoco_SOURCE_DIR}/lib/*.so*"
        "${mujoco_SOURCE_DIR}/lib/*.dylib"
        "${mujoco_SOURCE_DIR}/bin/*.dll"
        "${mujoco_SOURCE_DIR}/lib/*.dll")
    foreach(_rt IN LISTS MUJOCO_RUNTIME_FILES)
        add_custom_command(TARGET godot_mujoco POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy "${_rt}" "${GMJ_ADDON_BIN}/"
            COMMENT "Bundling MuJoCo runtime: ${_rt}")
    endforeach()
endif()
