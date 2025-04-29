
message(STATUS "FINDING LimeSuiteNG.")
if(NOT LIME_FOUND)
    pkg_check_modules (LimeSuiteNG_PKG LimeSuiteNG)

    find_path(LimeSuiteNG_INCLUDE_DIRS
            NAMES SDRDevice.h
            PATHS ${LimeSuiteNG_PKG_INCLUDE_DIRS}
            /usr/include/limesuiteng
            /usr/local/include/limesuiteng
            )

    find_library(LimeSuiteNG_LIBRARIES
            NAMES libLimeSuiteNG.so
            PATHS ${LimeSuiteNG_PKG_LIBRARY_DIRS}
            /usr/lib
            /usr/local/lib
            /usr/lib/arm-linux-gnueabihf
            )

    if(LimeSuiteNG_INCLUDE_DIRS AND LimeSuiteNG_LIBRARIES)
        set(LIME_FOUND TRUE CACHE INTERNAL "libLimeSuiteNG found")
        message(STATUS "Found libLimeSuiteNG: ${LimeSuiteNG_INCLUDE_DIRS}, ${LimeSuiteNG_LIBRARIES}")
    else(LimeSuiteNG_INCLUDE_DIRS AND LimeSuiteNG_LIBRARIES)
        set(LIME_FOUND FALSE CACHE INTERNAL "libLimeSuiteNG found")
        message(STATUS "libLimeSuiteNG not found.")
    endif(LimeSuiteNG_INCLUDE_DIRS AND LimeSuiteNG_LIBRARIES)

    mark_as_advanced(LimeSuiteNG_LIBRARIES LimeSuiteNG_INCLUDE_DIRS)

endif(NOT LIME_FOUND)
