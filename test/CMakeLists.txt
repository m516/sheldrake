cmake_minimum_required(VERSION 3.14)

# GoogleTest requires at least C++11
set(CMAKE_CXX_STANDARD 11)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

enable_testing()

add_executable(
  Sheldrake_tests
  basic_tests.cc
)
target_link_libraries(
  Sheldrake_tests
  gtest_main
  sheldrake
)

include(GoogleTest)
gtest_discover_tests(basic_tests)
