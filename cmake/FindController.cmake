

find_path(CONTROLLER_INCLUDE_DIR controller.h
        PATH_SUFFIXES include
        PATHS ${PROJECT_SOURCE_DIR}
        )


find_library(CONTROLLER_LIBRARY
        NAMES Controller
        PATH_SUFFIXES lib
        PATHS ${PROJECT_SOURCE_DIR}
        )

