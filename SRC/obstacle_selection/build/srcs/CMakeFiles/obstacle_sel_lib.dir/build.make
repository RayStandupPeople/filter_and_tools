# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/ze/Documents/LogAnalysis_Algos/SRC/obstacle_selection

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ze/Documents/LogAnalysis_Algos/SRC/obstacle_selection/build

# Include any dependencies generated for this target.
include srcs/CMakeFiles/obstacle_sel_lib.dir/depend.make

# Include the progress variables for this target.
include srcs/CMakeFiles/obstacle_sel_lib.dir/progress.make

# Include the compile flags for this target's objects.
include srcs/CMakeFiles/obstacle_sel_lib.dir/flags.make

srcs/CMakeFiles/obstacle_sel_lib.dir/obstacle_selection.cc.o: srcs/CMakeFiles/obstacle_sel_lib.dir/flags.make
srcs/CMakeFiles/obstacle_sel_lib.dir/obstacle_selection.cc.o: ../srcs/obstacle_selection.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ze/Documents/LogAnalysis_Algos/SRC/obstacle_selection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object srcs/CMakeFiles/obstacle_sel_lib.dir/obstacle_selection.cc.o"
	cd /home/ze/Documents/LogAnalysis_Algos/SRC/obstacle_selection/build/srcs && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/obstacle_sel_lib.dir/obstacle_selection.cc.o -c /home/ze/Documents/LogAnalysis_Algos/SRC/obstacle_selection/srcs/obstacle_selection.cc

srcs/CMakeFiles/obstacle_sel_lib.dir/obstacle_selection.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/obstacle_sel_lib.dir/obstacle_selection.cc.i"
	cd /home/ze/Documents/LogAnalysis_Algos/SRC/obstacle_selection/build/srcs && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ze/Documents/LogAnalysis_Algos/SRC/obstacle_selection/srcs/obstacle_selection.cc > CMakeFiles/obstacle_sel_lib.dir/obstacle_selection.cc.i

srcs/CMakeFiles/obstacle_sel_lib.dir/obstacle_selection.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/obstacle_sel_lib.dir/obstacle_selection.cc.s"
	cd /home/ze/Documents/LogAnalysis_Algos/SRC/obstacle_selection/build/srcs && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ze/Documents/LogAnalysis_Algos/SRC/obstacle_selection/srcs/obstacle_selection.cc -o CMakeFiles/obstacle_sel_lib.dir/obstacle_selection.cc.s

srcs/CMakeFiles/obstacle_sel_lib.dir/obstacle_selection.cc.o.requires:

.PHONY : srcs/CMakeFiles/obstacle_sel_lib.dir/obstacle_selection.cc.o.requires

srcs/CMakeFiles/obstacle_sel_lib.dir/obstacle_selection.cc.o.provides: srcs/CMakeFiles/obstacle_sel_lib.dir/obstacle_selection.cc.o.requires
	$(MAKE) -f srcs/CMakeFiles/obstacle_sel_lib.dir/build.make srcs/CMakeFiles/obstacle_sel_lib.dir/obstacle_selection.cc.o.provides.build
.PHONY : srcs/CMakeFiles/obstacle_sel_lib.dir/obstacle_selection.cc.o.provides

srcs/CMakeFiles/obstacle_sel_lib.dir/obstacle_selection.cc.o.provides.build: srcs/CMakeFiles/obstacle_sel_lib.dir/obstacle_selection.cc.o


# Object files for target obstacle_sel_lib
obstacle_sel_lib_OBJECTS = \
"CMakeFiles/obstacle_sel_lib.dir/obstacle_selection.cc.o"

# External object files for target obstacle_sel_lib
obstacle_sel_lib_EXTERNAL_OBJECTS =

srcs/libobstacle_sel_lib.a: srcs/CMakeFiles/obstacle_sel_lib.dir/obstacle_selection.cc.o
srcs/libobstacle_sel_lib.a: srcs/CMakeFiles/obstacle_sel_lib.dir/build.make
srcs/libobstacle_sel_lib.a: srcs/CMakeFiles/obstacle_sel_lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ze/Documents/LogAnalysis_Algos/SRC/obstacle_selection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libobstacle_sel_lib.a"
	cd /home/ze/Documents/LogAnalysis_Algos/SRC/obstacle_selection/build/srcs && $(CMAKE_COMMAND) -P CMakeFiles/obstacle_sel_lib.dir/cmake_clean_target.cmake
	cd /home/ze/Documents/LogAnalysis_Algos/SRC/obstacle_selection/build/srcs && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/obstacle_sel_lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
srcs/CMakeFiles/obstacle_sel_lib.dir/build: srcs/libobstacle_sel_lib.a

.PHONY : srcs/CMakeFiles/obstacle_sel_lib.dir/build

srcs/CMakeFiles/obstacle_sel_lib.dir/requires: srcs/CMakeFiles/obstacle_sel_lib.dir/obstacle_selection.cc.o.requires

.PHONY : srcs/CMakeFiles/obstacle_sel_lib.dir/requires

srcs/CMakeFiles/obstacle_sel_lib.dir/clean:
	cd /home/ze/Documents/LogAnalysis_Algos/SRC/obstacle_selection/build/srcs && $(CMAKE_COMMAND) -P CMakeFiles/obstacle_sel_lib.dir/cmake_clean.cmake
.PHONY : srcs/CMakeFiles/obstacle_sel_lib.dir/clean

srcs/CMakeFiles/obstacle_sel_lib.dir/depend:
	cd /home/ze/Documents/LogAnalysis_Algos/SRC/obstacle_selection/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ze/Documents/LogAnalysis_Algos/SRC/obstacle_selection /home/ze/Documents/LogAnalysis_Algos/SRC/obstacle_selection/srcs /home/ze/Documents/LogAnalysis_Algos/SRC/obstacle_selection/build /home/ze/Documents/LogAnalysis_Algos/SRC/obstacle_selection/build/srcs /home/ze/Documents/LogAnalysis_Algos/SRC/obstacle_selection/build/srcs/CMakeFiles/obstacle_sel_lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : srcs/CMakeFiles/obstacle_sel_lib.dir/depend

