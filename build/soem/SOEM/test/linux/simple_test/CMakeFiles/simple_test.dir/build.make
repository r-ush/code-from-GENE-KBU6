# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/gene/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gene/catkin_ws/build

# Include any dependencies generated for this target.
include soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/depend.make

# Include the progress variables for this target.
include soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/progress.make

# Include the compile flags for this target's objects.
include soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/flags.make

soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/simple_test.c.o: soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/flags.make
soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/simple_test.c.o: /home/gene/catkin_ws/src/soem/SOEM/test/linux/simple_test/simple_test.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gene/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/simple_test.c.o"
	cd /home/gene/catkin_ws/build/soem/SOEM/test/linux/simple_test && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/simple_test.dir/simple_test.c.o   -c /home/gene/catkin_ws/src/soem/SOEM/test/linux/simple_test/simple_test.c

soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/simple_test.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/simple_test.dir/simple_test.c.i"
	cd /home/gene/catkin_ws/build/soem/SOEM/test/linux/simple_test && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/gene/catkin_ws/src/soem/SOEM/test/linux/simple_test/simple_test.c > CMakeFiles/simple_test.dir/simple_test.c.i

soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/simple_test.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/simple_test.dir/simple_test.c.s"
	cd /home/gene/catkin_ws/build/soem/SOEM/test/linux/simple_test && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/gene/catkin_ws/src/soem/SOEM/test/linux/simple_test/simple_test.c -o CMakeFiles/simple_test.dir/simple_test.c.s

soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/simple_test.c.o.requires:

.PHONY : soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/simple_test.c.o.requires

soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/simple_test.c.o.provides: soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/simple_test.c.o.requires
	$(MAKE) -f soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/build.make soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/simple_test.c.o.provides.build
.PHONY : soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/simple_test.c.o.provides

soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/simple_test.c.o.provides.build: soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/simple_test.c.o


# Object files for target simple_test
simple_test_OBJECTS = \
"CMakeFiles/simple_test.dir/simple_test.c.o"

# External object files for target simple_test
simple_test_EXTERNAL_OBJECTS =

/home/gene/catkin_ws/devel/lib/soem/simple_test: soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/simple_test.c.o
/home/gene/catkin_ws/devel/lib/soem/simple_test: soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/build.make
/home/gene/catkin_ws/devel/lib/soem/simple_test: /home/gene/catkin_ws/devel/lib/libsoem.a
/home/gene/catkin_ws/devel/lib/soem/simple_test: soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/gene/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable /home/gene/catkin_ws/devel/lib/soem/simple_test"
	cd /home/gene/catkin_ws/build/soem/SOEM/test/linux/simple_test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simple_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/build: /home/gene/catkin_ws/devel/lib/soem/simple_test

.PHONY : soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/build

soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/requires: soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/simple_test.c.o.requires

.PHONY : soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/requires

soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/clean:
	cd /home/gene/catkin_ws/build/soem/SOEM/test/linux/simple_test && $(CMAKE_COMMAND) -P CMakeFiles/simple_test.dir/cmake_clean.cmake
.PHONY : soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/clean

soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/depend:
	cd /home/gene/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gene/catkin_ws/src /home/gene/catkin_ws/src/soem/SOEM/test/linux/simple_test /home/gene/catkin_ws/build /home/gene/catkin_ws/build/soem/SOEM/test/linux/simple_test /home/gene/catkin_ws/build/soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : soem/SOEM/test/linux/simple_test/CMakeFiles/simple_test.dir/depend
