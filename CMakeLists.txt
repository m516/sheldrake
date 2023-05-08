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

# Set the macro/helper directory 
set(MACRO_DIRECTORY ${CMAKE_HOME_DIRECTORY}/CMake/macros)
message("${BoldMagenta}CMake utility directory: ${MACRO_DIRECTORY}${ColorReset}")
# Add some macros
include(${MACRO_DIRECTORY}/libhelpers.cmake)

# Find packages used throughout this repository
message("${BoldYellow}Searching for libraries...${ColorReset}")

# Drake
find_package(drake)


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