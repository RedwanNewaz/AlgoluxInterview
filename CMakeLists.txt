cmake_minimum_required(VERSION 3.10)
project(localization)
set(CMAKE_CXX_STANDARD 11)
list(APPEND CMAKE_EXE_LINKER_FLAGS "-no-pie")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Controller REQUIRED)
find_package(OpenGL)
find_package(GLUT)

include_directories(
        include
        /usr/include
        )
link_directories(/usr/lib/x86_64-linux-gnu/)
add_executable(localization main.cpp)
message(STATUS Hi ${CONTROLLER_LIBRARY})
target_link_libraries(localization
        ${CONTROLLER_LIBRARY}
        ${OPENGL_gl_LIBRARY}
        ${GLUT_LIBRARIES}
        )

