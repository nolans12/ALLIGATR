# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ckohl10/ALLIGATR/HUB

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ckohl10/ALLIGATR/HUB/build

# Include any dependencies generated for this target.
include CMakeFiles/main.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/main.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/main.dir/flags.make

CMakeFiles/main.dir/src/CameraObject.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/CameraObject.cpp.o: ../src/CameraObject.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ckohl10/ALLIGATR/HUB/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/main.dir/src/CameraObject.cpp.o"
	/usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/CameraObject.cpp.o -c /home/ckohl10/ALLIGATR/HUB/src/CameraObject.cpp

CMakeFiles/main.dir/src/CameraObject.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/CameraObject.cpp.i"
	/usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ckohl10/ALLIGATR/HUB/src/CameraObject.cpp > CMakeFiles/main.dir/src/CameraObject.cpp.i

CMakeFiles/main.dir/src/CameraObject.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/CameraObject.cpp.s"
	/usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ckohl10/ALLIGATR/HUB/src/CameraObject.cpp -o CMakeFiles/main.dir/src/CameraObject.cpp.s

CMakeFiles/main.dir/src/HUB.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/HUB.cpp.o: ../src/HUB.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ckohl10/ALLIGATR/HUB/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/main.dir/src/HUB.cpp.o"
	/usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/HUB.cpp.o -c /home/ckohl10/ALLIGATR/HUB/src/HUB.cpp

CMakeFiles/main.dir/src/HUB.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/HUB.cpp.i"
	/usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ckohl10/ALLIGATR/HUB/src/HUB.cpp > CMakeFiles/main.dir/src/HUB.cpp.i

CMakeFiles/main.dir/src/HUB.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/HUB.cpp.s"
	/usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ckohl10/ALLIGATR/HUB/src/HUB.cpp -o CMakeFiles/main.dir/src/HUB.cpp.s

CMakeFiles/main.dir/src/WIFI.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/WIFI.cpp.o: ../src/WIFI.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ckohl10/ALLIGATR/HUB/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/main.dir/src/WIFI.cpp.o"
	/usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/WIFI.cpp.o -c /home/ckohl10/ALLIGATR/HUB/src/WIFI.cpp

CMakeFiles/main.dir/src/WIFI.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/WIFI.cpp.i"
	/usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ckohl10/ALLIGATR/HUB/src/WIFI.cpp > CMakeFiles/main.dir/src/WIFI.cpp.i

CMakeFiles/main.dir/src/WIFI.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/WIFI.cpp.s"
	/usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ckohl10/ALLIGATR/HUB/src/WIFI.cpp -o CMakeFiles/main.dir/src/WIFI.cpp.s

CMakeFiles/main.dir/src/main.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ckohl10/ALLIGATR/HUB/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/main.dir/src/main.cpp.o"
	/usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/main.cpp.o -c /home/ckohl10/ALLIGATR/HUB/src/main.cpp

CMakeFiles/main.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/main.cpp.i"
	/usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ckohl10/ALLIGATR/HUB/src/main.cpp > CMakeFiles/main.dir/src/main.cpp.i

CMakeFiles/main.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/main.cpp.s"
	/usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ckohl10/ALLIGATR/HUB/src/main.cpp -o CMakeFiles/main.dir/src/main.cpp.s

CMakeFiles/main.dir/src/pathStep.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/pathStep.cpp.o: ../src/pathStep.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ckohl10/ALLIGATR/HUB/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/main.dir/src/pathStep.cpp.o"
	/usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/pathStep.cpp.o -c /home/ckohl10/ALLIGATR/HUB/src/pathStep.cpp

CMakeFiles/main.dir/src/pathStep.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/pathStep.cpp.i"
	/usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ckohl10/ALLIGATR/HUB/src/pathStep.cpp > CMakeFiles/main.dir/src/pathStep.cpp.i

CMakeFiles/main.dir/src/pathStep.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/pathStep.cpp.s"
	/usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ckohl10/ALLIGATR/HUB/src/pathStep.cpp -o CMakeFiles/main.dir/src/pathStep.cpp.s

CMakeFiles/main.dir/src/phaseStep.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/phaseStep.cpp.o: ../src/phaseStep.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ckohl10/ALLIGATR/HUB/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/main.dir/src/phaseStep.cpp.o"
	/usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/phaseStep.cpp.o -c /home/ckohl10/ALLIGATR/HUB/src/phaseStep.cpp

CMakeFiles/main.dir/src/phaseStep.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/phaseStep.cpp.i"
	/usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ckohl10/ALLIGATR/HUB/src/phaseStep.cpp > CMakeFiles/main.dir/src/phaseStep.cpp.i

CMakeFiles/main.dir/src/phaseStep.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/phaseStep.cpp.s"
	/usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ckohl10/ALLIGATR/HUB/src/phaseStep.cpp -o CMakeFiles/main.dir/src/phaseStep.cpp.s

# Object files for target main
main_OBJECTS = \
"CMakeFiles/main.dir/src/CameraObject.cpp.o" \
"CMakeFiles/main.dir/src/HUB.cpp.o" \
"CMakeFiles/main.dir/src/WIFI.cpp.o" \
"CMakeFiles/main.dir/src/main.cpp.o" \
"CMakeFiles/main.dir/src/pathStep.cpp.o" \
"CMakeFiles/main.dir/src/phaseStep.cpp.o"

# External object files for target main
main_EXTERNAL_OBJECTS =

main: CMakeFiles/main.dir/src/CameraObject.cpp.o
main: CMakeFiles/main.dir/src/HUB.cpp.o
main: CMakeFiles/main.dir/src/WIFI.cpp.o
main: CMakeFiles/main.dir/src/main.cpp.o
main: CMakeFiles/main.dir/src/pathStep.cpp.o
main: CMakeFiles/main.dir/src/phaseStep.cpp.o
main: CMakeFiles/main.dir/build.make
main: CMakeFiles/main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ckohl10/ALLIGATR/HUB/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable main"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/main.dir/build: main

.PHONY : CMakeFiles/main.dir/build

CMakeFiles/main.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/main.dir/cmake_clean.cmake
.PHONY : CMakeFiles/main.dir/clean

CMakeFiles/main.dir/depend:
	cd /home/ckohl10/ALLIGATR/HUB/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ckohl10/ALLIGATR/HUB /home/ckohl10/ALLIGATR/HUB /home/ckohl10/ALLIGATR/HUB/build /home/ckohl10/ALLIGATR/HUB/build /home/ckohl10/ALLIGATR/HUB/build/CMakeFiles/main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/main.dir/depend
