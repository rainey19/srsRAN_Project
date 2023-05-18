
message(STATUS "FINDING LimeSuite2.")
if(NOT LIME_FOUND)
    pkg_check_modules (LimeSuite2_PKG LimeSuite2)

    find_path(LimeSuite2_INCLUDE_DIRS
            NAMES SDRDevice.h
            PATHS ${LimeSuite2_PKG_INCLUDE_DIRS}
            /usr/include/limesuite
            /usr/local/include/limesuite
            )

    find_library(LimeSuite2_LIBRARIES
            NAMES libLimeSuite2.so
            PATHS ${LimeSuite2_PKG_LIBRARY_DIRS}
            /usr/lib
            /usr/local/lib
            )

    if(LimeSuite2_INCLUDE_DIRS AND LimeSuite2_LIBRARIES)
        set(LIME_FOUND TRUE CACHE INTERNAL "libLimeSuite2 found")
        message(STATUS "Found libLimeSuite2: ${LimeSuite2_INCLUDE_DIRS}, ${LimeSuite2_LIBRARIES}")
    else(LimeSuite2_INCLUDE_DIRS AND LimeSuite2_LIBRARIES)
        set(LIME_FOUND FALSE CACHE INTERNAL "libLimeSuite2 found")
        message(STATUS "libLimeSuite2 not found.")
    endif(LimeSuite2_INCLUDE_DIRS AND LimeSuite2_LIBRARIES)

    mark_as_advanced(LimeSuite2_LIBRARIES LimeSuite2_INCLUDE_DIRS)

endif(NOT LIME_FOUND)
