# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.

cmake_minimum_required (VERSION 3.10)

# Options
option(BUILD_TESTS "Build all projects in the 'test' folder" ON)
option(BUILD_EXAMPLES "Build all projects in the 'examples' folder" ON)


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


#########################   Libraries   #########################


# Find packages used throughout this repository
message("${BoldYellow}Searching for libraries...${ColorReset}")

# Drake
find_package(drake)
if(drake_FOUND)
  message("${Green}Drake found. Including Drake integrations.${ColorReset}")
  add_compile_definitions(USING_DRAKE_LIBRARY=1)
else()
  message("${BoldRed}Drake not found. Not including Drake integration in this build.${ColorReset}")
endif()

# GFlags
add_subdirectory(thirdparty/gflags)

# TinyMatrixMath
add_subdirectory(thirdparty/TinyMatrixMath)


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
target_link_libraries (serial rt pthread)
target_include_directories(serial PUBLIC thirdparty/serial/include/)


message("${BoldYellow}Library search complete!${ColorReset}")


#########################   Project   #########################


set(PROJECT_SOURCES
    src/sheldrake.cpp
    include/sheldrake.hpp
)
add_library (sheldrake ${PROJECT_SOURCES})
target_link_libraries (sheldrake tinycontrolpolicies serializer)
if(drake_FOUND)
  target_link_libraries (sheldrake drake::drake)
endif()
target_include_directories(sheldrake PRIVATE thirdparty/serial/include/)
target_include_directories(sheldrake PUBLIC include/)


#########################   Tests   #########################


# Include tests.
if(BUILD_TESTS)
include(CTest)
add_subdirectory ("test")
endif()

if(BUILD_EXAMPLES)
add_subdirectory(examples)
endif()