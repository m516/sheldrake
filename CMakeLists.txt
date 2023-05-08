# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.

cmake_minimum_required (VERSION 3.10)

# Define colors
if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColorReset "${Esc}[m")
  set(ColorBold  "${Esc}[1m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
  set(Yellow      "${Esc}[33m")
  set(Blue        "${Esc}[34m")
  set(Magenta     "${Esc}[35m")
  set(Cyan        "${Esc}[36m")
  set(White       "${Esc}[37m")
  set(BoldRed     "${Esc}[1;31m")
  set(BoldGreen   "${Esc}[1;32m")
  set(BoldYellow  "${Esc}[1;33m")
  set(BoldBlue    "${Esc}[1;34m")
  set(BoldMagenta "${Esc}[1;35m")
  set(BoldCyan    "${Esc}[1;36m")
  set(BoldWhite   "${Esc}[1;37m")
endif()

project ("Sheldrake" C CXX)
set (CMAKE_CXX_STANDARD 20) # We use new features like std::endian, std::float_32, ...

# Set the macro/helper directory 
set(MACRO_DIRECTORY ${CMAKE_HOME_DIRECTORY}/CMake/macros)
message("${BoldMagenta}CMake utility directory: ${MACRO_DIRECTORY}${ColorReset}")
# Add some macros
include(${MACRO_DIRECTORY}/libhelpers.cmake)

# Find packages used throughout this repository
message("${BoldYellow}Searching for libraries...${ColorReset}")

# Drake
find_package(drake)


# Serial
# add_subdirectory(thirdparty/serial)
# The Serial cmake file is not meant for this configuration,
# so we remake our own.
add_library (serial ${PROJECT_SOURCES}
thirdparty/serial/include/serial/impl
thirdparty/serial/src/impl/list_ports/list_ports_linux.cc
thirdparty/serial/src/impl/unix.cc
thirdparty/serial/include/serial/impl/unix.h
thirdparty/serial/include/serial/serial.h
thirdparty/serial/include/serial/v8stdint.h
thirdparty/serial/src/serial.cc
)
target_link_libraries (serial rt pthreads)
target_include_directories(serial PUBLIC thirdparty/serial/include/)


# Google Test
# message("${BoldYellow}Using local copy of the Google Test framework${ColorReset}")
# add_subdirectory(thirdparty/googletest)
# set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)


message("${BoldYellow}Library search complete!${ColorReset}")

# Include sub-projects.
include_directories("src")
add_subdirectory ("src")

# Include tests.
include(CTest)
add_subdirectory ("test")