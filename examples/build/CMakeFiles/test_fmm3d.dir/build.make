# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/acerillo/fastmarching/examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/acerillo/fastmarching/examples/build

# Include any dependencies generated for this target.
include CMakeFiles/test_fmm3d.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_fmm3d.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_fmm3d.dir/flags.make

CMakeFiles/test_fmm3d.dir/test_fmm3d.cpp.o: CMakeFiles/test_fmm3d.dir/flags.make
CMakeFiles/test_fmm3d.dir/test_fmm3d.cpp.o: ../test_fmm3d.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/acerillo/fastmarching/examples/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/test_fmm3d.dir/test_fmm3d.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_fmm3d.dir/test_fmm3d.cpp.o -c /home/acerillo/fastmarching/examples/test_fmm3d.cpp

CMakeFiles/test_fmm3d.dir/test_fmm3d.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_fmm3d.dir/test_fmm3d.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/acerillo/fastmarching/examples/test_fmm3d.cpp > CMakeFiles/test_fmm3d.dir/test_fmm3d.cpp.i

CMakeFiles/test_fmm3d.dir/test_fmm3d.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_fmm3d.dir/test_fmm3d.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/acerillo/fastmarching/examples/test_fmm3d.cpp -o CMakeFiles/test_fmm3d.dir/test_fmm3d.cpp.s

CMakeFiles/test_fmm3d.dir/test_fmm3d.cpp.o.requires:
.PHONY : CMakeFiles/test_fmm3d.dir/test_fmm3d.cpp.o.requires

CMakeFiles/test_fmm3d.dir/test_fmm3d.cpp.o.provides: CMakeFiles/test_fmm3d.dir/test_fmm3d.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_fmm3d.dir/build.make CMakeFiles/test_fmm3d.dir/test_fmm3d.cpp.o.provides.build
.PHONY : CMakeFiles/test_fmm3d.dir/test_fmm3d.cpp.o.provides

CMakeFiles/test_fmm3d.dir/test_fmm3d.cpp.o.provides.build: CMakeFiles/test_fmm3d.dir/test_fmm3d.cpp.o

CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/console/console.cpp.o: CMakeFiles/test_fmm3d.dir/flags.make
CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/console/console.cpp.o: /home/acerillo/fastmarching/console/console.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/acerillo/fastmarching/examples/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/console/console.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/console/console.cpp.o -c /home/acerillo/fastmarching/console/console.cpp

CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/console/console.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/console/console.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/acerillo/fastmarching/console/console.cpp > CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/console/console.cpp.i

CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/console/console.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/console/console.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/acerillo/fastmarching/console/console.cpp -o CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/console/console.cpp.s

CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/console/console.cpp.o.requires:
.PHONY : CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/console/console.cpp.o.requires

CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/console/console.cpp.o.provides: CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/console/console.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_fmm3d.dir/build.make CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/console/console.cpp.o.provides.build
.PHONY : CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/console/console.cpp.o.provides

CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/console/console.cpp.o.provides.build: CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/console/console.cpp.o

CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/ndgridmap/cell.cpp.o: CMakeFiles/test_fmm3d.dir/flags.make
CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/ndgridmap/cell.cpp.o: /home/acerillo/fastmarching/ndgridmap/cell.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/acerillo/fastmarching/examples/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/ndgridmap/cell.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/ndgridmap/cell.cpp.o -c /home/acerillo/fastmarching/ndgridmap/cell.cpp

CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/ndgridmap/cell.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/ndgridmap/cell.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/acerillo/fastmarching/ndgridmap/cell.cpp > CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/ndgridmap/cell.cpp.i

CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/ndgridmap/cell.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/ndgridmap/cell.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/acerillo/fastmarching/ndgridmap/cell.cpp -o CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/ndgridmap/cell.cpp.s

CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/ndgridmap/cell.cpp.o.requires:
.PHONY : CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/ndgridmap/cell.cpp.o.requires

CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/ndgridmap/cell.cpp.o.provides: CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/ndgridmap/cell.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_fmm3d.dir/build.make CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/ndgridmap/cell.cpp.o.provides.build
.PHONY : CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/ndgridmap/cell.cpp.o.provides

CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/ndgridmap/cell.cpp.o.provides.build: CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/ndgridmap/cell.cpp.o

CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/fmm/fmdata/fmcell.cpp.o: CMakeFiles/test_fmm3d.dir/flags.make
CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/fmm/fmdata/fmcell.cpp.o: /home/acerillo/fastmarching/fmm/fmdata/fmcell.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/acerillo/fastmarching/examples/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/fmm/fmdata/fmcell.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/fmm/fmdata/fmcell.cpp.o -c /home/acerillo/fastmarching/fmm/fmdata/fmcell.cpp

CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/fmm/fmdata/fmcell.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/fmm/fmdata/fmcell.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/acerillo/fastmarching/fmm/fmdata/fmcell.cpp > CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/fmm/fmdata/fmcell.cpp.i

CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/fmm/fmdata/fmcell.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/fmm/fmdata/fmcell.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/acerillo/fastmarching/fmm/fmdata/fmcell.cpp -o CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/fmm/fmdata/fmcell.cpp.s

CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/fmm/fmdata/fmcell.cpp.o.requires:
.PHONY : CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/fmm/fmdata/fmcell.cpp.o.requires

CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/fmm/fmdata/fmcell.cpp.o.provides: CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/fmm/fmdata/fmcell.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_fmm3d.dir/build.make CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/fmm/fmdata/fmcell.cpp.o.provides.build
.PHONY : CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/fmm/fmdata/fmcell.cpp.o.provides

CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/fmm/fmdata/fmcell.cpp.o.provides.build: CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/fmm/fmdata/fmcell.cpp.o

# Object files for target test_fmm3d
test_fmm3d_OBJECTS = \
"CMakeFiles/test_fmm3d.dir/test_fmm3d.cpp.o" \
"CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/console/console.cpp.o" \
"CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/ndgridmap/cell.cpp.o" \
"CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/fmm/fmdata/fmcell.cpp.o"

# External object files for target test_fmm3d
test_fmm3d_EXTERNAL_OBJECTS =

test_fmm3d: CMakeFiles/test_fmm3d.dir/test_fmm3d.cpp.o
test_fmm3d: CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/console/console.cpp.o
test_fmm3d: CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/ndgridmap/cell.cpp.o
test_fmm3d: CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/fmm/fmdata/fmcell.cpp.o
test_fmm3d: CMakeFiles/test_fmm3d.dir/build.make
test_fmm3d: CMakeFiles/test_fmm3d.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable test_fmm3d"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_fmm3d.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_fmm3d.dir/build: test_fmm3d
.PHONY : CMakeFiles/test_fmm3d.dir/build

CMakeFiles/test_fmm3d.dir/requires: CMakeFiles/test_fmm3d.dir/test_fmm3d.cpp.o.requires
CMakeFiles/test_fmm3d.dir/requires: CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/console/console.cpp.o.requires
CMakeFiles/test_fmm3d.dir/requires: CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/ndgridmap/cell.cpp.o.requires
CMakeFiles/test_fmm3d.dir/requires: CMakeFiles/test_fmm3d.dir/home/acerillo/fastmarching/fmm/fmdata/fmcell.cpp.o.requires
.PHONY : CMakeFiles/test_fmm3d.dir/requires

CMakeFiles/test_fmm3d.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_fmm3d.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_fmm3d.dir/clean

CMakeFiles/test_fmm3d.dir/depend:
	cd /home/acerillo/fastmarching/examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/acerillo/fastmarching/examples /home/acerillo/fastmarching/examples /home/acerillo/fastmarching/examples/build /home/acerillo/fastmarching/examples/build /home/acerillo/fastmarching/examples/build/CMakeFiles/test_fmm3d.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_fmm3d.dir/depend
