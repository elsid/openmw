find_path(DETOUR_INCLUDE_DIR
    NAMES DetourCommon.h
    HINTS $ENV{DETOUR_ROOT}
        ${DETOUR_ROOT}
    PATH_SUFFIXES include
)
mark_as_advanced(DETOUR_INCLUDE_DIR)

find_library(DETOUR_LIBRARY
    NAMES Detour
    HINTS $ENV{DETOUR_ROOT}
        ${DETOUR_ROOT}
    PATH_SUFFIXES lib
)
mark_as_advanced(DETOUR_LIBRARY)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Detour DEFAULT_MSG DETOUR_LIBRARY DETOUR_INCLUDE_DIR)

if(DETOUR_FOUND)
    set(DETOUR_INCLUDE_DIRS ${DETOUR_INCLUDE_DIR})
    set(DETOUR_LIBRARIES ${DETOUR_LIBRARY})
endif()
