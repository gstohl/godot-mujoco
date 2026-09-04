# Apply PATCH_FILE with `patch -p1 -N` in CMAKE_CURRENT_SOURCE_DIR
# (FetchContent runs PATCH_COMMAND in the extracted source dir).
# Exit 0 = applied, 1 = already applied / skipped hunks; anything else fails.
if(NOT PATCH_FILE)
    message(FATAL_ERROR "apply_patch.cmake: PATCH_FILE is required")
endif()
execute_process(
    COMMAND patch -p1 -N -i "${PATCH_FILE}"
    RESULT_VARIABLE _rc
    OUTPUT_VARIABLE _out
    ERROR_VARIABLE _err)
message(STATUS "${_out}${_err}")
if(_rc GREATER 1)
    message(FATAL_ERROR "patch failed (exit ${_rc}): ${PATCH_FILE}")
endif()
