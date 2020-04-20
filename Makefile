# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.17.1/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.17.1/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/Users/jakebliss/College /fourth_year/ar/hw2"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/Users/jakebliss/College /fourth_year/ar/hw2"

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/local/Cellar/cmake/3.17.1/bin/cmake --regenerate-during-build -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/local/Cellar/cmake/3.17.1/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start "/Users/jakebliss/College /fourth_year/ar/hw2/CMakeFiles" "/Users/jakebliss/College /fourth_year/ar/hw2/CMakeFiles/progress.marks"
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start "/Users/jakebliss/College /fourth_year/ar/hw2/CMakeFiles" 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named camera_calibration

# Build rule for target.
camera_calibration: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 camera_calibration
.PHONY : camera_calibration

# fast build rule for target.
camera_calibration/fast:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/camera_calibration.dir/build.make CMakeFiles/camera_calibration.dir/build
.PHONY : camera_calibration/fast

#=============================================================================
# Target rules for targets named opengl_cv_skeleton

# Build rule for target.
opengl_cv_skeleton: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 opengl_cv_skeleton
.PHONY : opengl_cv_skeleton

# fast build rule for target.
opengl_cv_skeleton/fast:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/opengl_cv_skeleton.dir/build.make CMakeFiles/opengl_cv_skeleton.dir/build
.PHONY : opengl_cv_skeleton/fast

camera_calibration.o: camera_calibration.cpp.o

.PHONY : camera_calibration.o

# target to build an object file
camera_calibration.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/camera_calibration.dir/build.make CMakeFiles/camera_calibration.dir/camera_calibration.cpp.o
.PHONY : camera_calibration.cpp.o

camera_calibration.i: camera_calibration.cpp.i

.PHONY : camera_calibration.i

# target to preprocess a source file
camera_calibration.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/camera_calibration.dir/build.make CMakeFiles/camera_calibration.dir/camera_calibration.cpp.i
.PHONY : camera_calibration.cpp.i

camera_calibration.s: camera_calibration.cpp.s

.PHONY : camera_calibration.s

# target to generate assembly for a file
camera_calibration.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/camera_calibration.dir/build.make CMakeFiles/camera_calibration.dir/camera_calibration.cpp.s
.PHONY : camera_calibration.cpp.s

opengl_cv_skeleton.o: opengl_cv_skeleton.cpp.o

.PHONY : opengl_cv_skeleton.o

# target to build an object file
opengl_cv_skeleton.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/opengl_cv_skeleton.dir/build.make CMakeFiles/opengl_cv_skeleton.dir/opengl_cv_skeleton.cpp.o
.PHONY : opengl_cv_skeleton.cpp.o

opengl_cv_skeleton.i: opengl_cv_skeleton.cpp.i

.PHONY : opengl_cv_skeleton.i

# target to preprocess a source file
opengl_cv_skeleton.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/opengl_cv_skeleton.dir/build.make CMakeFiles/opengl_cv_skeleton.dir/opengl_cv_skeleton.cpp.i
.PHONY : opengl_cv_skeleton.cpp.i

opengl_cv_skeleton.s: opengl_cv_skeleton.cpp.s

.PHONY : opengl_cv_skeleton.s

# target to generate assembly for a file
opengl_cv_skeleton.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/opengl_cv_skeleton.dir/build.make CMakeFiles/opengl_cv_skeleton.dir/opengl_cv_skeleton.cpp.s
.PHONY : opengl_cv_skeleton.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... edit_cache"
	@echo "... rebuild_cache"
	@echo "... camera_calibration"
	@echo "... opengl_cv_skeleton"
	@echo "... camera_calibration.o"
	@echo "... camera_calibration.i"
	@echo "... camera_calibration.s"
	@echo "... opengl_cv_skeleton.o"
	@echo "... opengl_cv_skeleton.i"
	@echo "... opengl_cv_skeleton.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

