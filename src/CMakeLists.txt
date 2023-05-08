cmake_minimum_required (VERSION 3.8)

# In UNIX, set PROJECT_SOURCES using the following command
# find . | grep .cpp
# Delete the ./ at the beginning of each file with block selection (alt + shift in Visual Studio)
set(PROJECT_SOURCES
    test.cpp
)

# In UNIX, set PROJECT_HEADERS using the following command
# find . | grep .hpp
# Delete the ./ at the beginning of each file with block selection (alt + shift in Visual Studio)

# Add source to this project's executable.
add_library (sheldrake ${PROJECT_SOURCES})

# Add tests and install targets if needed.
target_link_libraries (sheldrake drake::drake) 
# target_include_directories(${EXECUTABLE_NAME} PUBLIC src)
