cmake_minimum_required(VERSION 3.10)
project(main)

# # Find OpenCV
# find_package(OpenCV REQUIRED)

# # Check if the OpenCV library was found
# if (NOT OpenCV_FOUND)
#     message(FATAL_ERROR "OpenCV not found!")
# endif()

# Include the OpenCV directories and the src directory
#include_directories(${OpenCV_INCLUDE_DIRS} src)

# Add a file glob to include all cpp files in the src directory
file(GLOB SRC_FILES "src/*.cpp")

# Create a library from the source files in src/
#add_library(helper_functions STATIC ${SRC_FILES})

add_executable(main ${SOURCE_FILE})

# Link the OpenCV library to the executable
#target_link_libraries(main ${OpenCV_LIBS})
#target_link_libraries(main ${OpenCV_LIBS})
