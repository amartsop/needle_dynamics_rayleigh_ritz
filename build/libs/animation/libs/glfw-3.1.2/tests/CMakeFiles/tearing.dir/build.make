# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /opt/cmake-3.18.4-Linux-x86_64/bin/cmake

# The command to remove a file.
RM = /opt/cmake-3.18.4-Linux-x86_64/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/thanos/Desktop/Rayleigh_Ritz

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/thanos/Desktop/Rayleigh_Ritz/build

# Include any dependencies generated for this target.
include libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/depend.make

# Include the progress variables for this target.
include libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/progress.make

# Include the compile flags for this target's objects.
include libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/flags.make

libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/tearing.c.o: libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/flags.make
libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/tearing.c.o: ../libs/animation/libs/glfw-3.1.2/tests/tearing.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/thanos/Desktop/Rayleigh_Ritz/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/tearing.c.o"
	cd /home/thanos/Desktop/Rayleigh_Ritz/build/libs/animation/libs/glfw-3.1.2/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/tearing.dir/tearing.c.o -c /home/thanos/Desktop/Rayleigh_Ritz/libs/animation/libs/glfw-3.1.2/tests/tearing.c

libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/tearing.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/tearing.dir/tearing.c.i"
	cd /home/thanos/Desktop/Rayleigh_Ritz/build/libs/animation/libs/glfw-3.1.2/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/thanos/Desktop/Rayleigh_Ritz/libs/animation/libs/glfw-3.1.2/tests/tearing.c > CMakeFiles/tearing.dir/tearing.c.i

libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/tearing.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/tearing.dir/tearing.c.s"
	cd /home/thanos/Desktop/Rayleigh_Ritz/build/libs/animation/libs/glfw-3.1.2/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/thanos/Desktop/Rayleigh_Ritz/libs/animation/libs/glfw-3.1.2/tests/tearing.c -o CMakeFiles/tearing.dir/tearing.c.s

libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/__/deps/getopt.c.o: libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/flags.make
libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/__/deps/getopt.c.o: ../libs/animation/libs/glfw-3.1.2/deps/getopt.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/thanos/Desktop/Rayleigh_Ritz/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/__/deps/getopt.c.o"
	cd /home/thanos/Desktop/Rayleigh_Ritz/build/libs/animation/libs/glfw-3.1.2/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/tearing.dir/__/deps/getopt.c.o -c /home/thanos/Desktop/Rayleigh_Ritz/libs/animation/libs/glfw-3.1.2/deps/getopt.c

libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/__/deps/getopt.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/tearing.dir/__/deps/getopt.c.i"
	cd /home/thanos/Desktop/Rayleigh_Ritz/build/libs/animation/libs/glfw-3.1.2/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/thanos/Desktop/Rayleigh_Ritz/libs/animation/libs/glfw-3.1.2/deps/getopt.c > CMakeFiles/tearing.dir/__/deps/getopt.c.i

libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/__/deps/getopt.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/tearing.dir/__/deps/getopt.c.s"
	cd /home/thanos/Desktop/Rayleigh_Ritz/build/libs/animation/libs/glfw-3.1.2/tests && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/thanos/Desktop/Rayleigh_Ritz/libs/animation/libs/glfw-3.1.2/deps/getopt.c -o CMakeFiles/tearing.dir/__/deps/getopt.c.s

# Object files for target tearing
tearing_OBJECTS = \
"CMakeFiles/tearing.dir/tearing.c.o" \
"CMakeFiles/tearing.dir/__/deps/getopt.c.o"

# External object files for target tearing
tearing_EXTERNAL_OBJECTS =

libs/animation/libs/glfw-3.1.2/tests/tearing: libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/tearing.c.o
libs/animation/libs/glfw-3.1.2/tests/tearing: libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/__/deps/getopt.c.o
libs/animation/libs/glfw-3.1.2/tests/tearing: libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/build.make
libs/animation/libs/glfw-3.1.2/tests/tearing: libs/animation/libs/glfw-3.1.2/src/libglfw3.a
libs/animation/libs/glfw-3.1.2/tests/tearing: /usr/lib/x86_64-linux-gnu/librt.so
libs/animation/libs/glfw-3.1.2/tests/tearing: /usr/lib/x86_64-linux-gnu/libm.so
libs/animation/libs/glfw-3.1.2/tests/tearing: /usr/lib/x86_64-linux-gnu/libX11.so
libs/animation/libs/glfw-3.1.2/tests/tearing: /usr/lib/x86_64-linux-gnu/libXrandr.so
libs/animation/libs/glfw-3.1.2/tests/tearing: /usr/lib/x86_64-linux-gnu/libXinerama.so
libs/animation/libs/glfw-3.1.2/tests/tearing: /usr/lib/x86_64-linux-gnu/libXi.so
libs/animation/libs/glfw-3.1.2/tests/tearing: /usr/lib/x86_64-linux-gnu/libXxf86vm.so
libs/animation/libs/glfw-3.1.2/tests/tearing: /usr/lib/x86_64-linux-gnu/libXcursor.so
libs/animation/libs/glfw-3.1.2/tests/tearing: /usr/lib/x86_64-linux-gnu/libGL.so
libs/animation/libs/glfw-3.1.2/tests/tearing: libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/thanos/Desktop/Rayleigh_Ritz/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C executable tearing"
	cd /home/thanos/Desktop/Rayleigh_Ritz/build/libs/animation/libs/glfw-3.1.2/tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tearing.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/build: libs/animation/libs/glfw-3.1.2/tests/tearing

.PHONY : libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/build

libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/clean:
	cd /home/thanos/Desktop/Rayleigh_Ritz/build/libs/animation/libs/glfw-3.1.2/tests && $(CMAKE_COMMAND) -P CMakeFiles/tearing.dir/cmake_clean.cmake
.PHONY : libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/clean

libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/depend:
	cd /home/thanos/Desktop/Rayleigh_Ritz/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/thanos/Desktop/Rayleigh_Ritz /home/thanos/Desktop/Rayleigh_Ritz/libs/animation/libs/glfw-3.1.2/tests /home/thanos/Desktop/Rayleigh_Ritz/build /home/thanos/Desktop/Rayleigh_Ritz/build/libs/animation/libs/glfw-3.1.2/tests /home/thanos/Desktop/Rayleigh_Ritz/build/libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : libs/animation/libs/glfw-3.1.2/tests/CMakeFiles/tearing.dir/depend

