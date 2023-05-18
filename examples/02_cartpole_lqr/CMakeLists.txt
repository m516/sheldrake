# This is the name of the executable
set(EXECUTABLE_NAME 02_cartpole_lqr)

# Add source to this project's executable.
add_executable (${EXECUTABLE_NAME} "02_cartpole_lqr.cpp")

# Add tests and install targets if needed.
TARGET_LINK_LIBRARIES (${EXECUTABLE_NAME} sheldrake gflags)