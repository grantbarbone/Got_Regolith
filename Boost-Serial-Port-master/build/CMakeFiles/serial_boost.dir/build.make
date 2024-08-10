# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.30

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/rover/Boost-Serial-Port-master

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rover/Boost-Serial-Port-master/build

# Include any dependencies generated for this target.
include CMakeFiles/serial_boost.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/serial_boost.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/serial_boost.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/serial_boost.dir/flags.make

CMakeFiles/serial_boost.dir/BoostSerial.cpp.o: CMakeFiles/serial_boost.dir/flags.make
CMakeFiles/serial_boost.dir/BoostSerial.cpp.o: /home/rover/Boost-Serial-Port-master/BoostSerial.cpp
CMakeFiles/serial_boost.dir/BoostSerial.cpp.o: CMakeFiles/serial_boost.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/rover/Boost-Serial-Port-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/serial_boost.dir/BoostSerial.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/serial_boost.dir/BoostSerial.cpp.o -MF CMakeFiles/serial_boost.dir/BoostSerial.cpp.o.d -o CMakeFiles/serial_boost.dir/BoostSerial.cpp.o -c /home/rover/Boost-Serial-Port-master/BoostSerial.cpp

CMakeFiles/serial_boost.dir/BoostSerial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/serial_boost.dir/BoostSerial.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rover/Boost-Serial-Port-master/BoostSerial.cpp > CMakeFiles/serial_boost.dir/BoostSerial.cpp.i

CMakeFiles/serial_boost.dir/BoostSerial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/serial_boost.dir/BoostSerial.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rover/Boost-Serial-Port-master/BoostSerial.cpp -o CMakeFiles/serial_boost.dir/BoostSerial.cpp.s

# Object files for target serial_boost
serial_boost_OBJECTS = \
"CMakeFiles/serial_boost.dir/BoostSerial.cpp.o"

# External object files for target serial_boost
serial_boost_EXTERNAL_OBJECTS =

libserial_boost.a: CMakeFiles/serial_boost.dir/BoostSerial.cpp.o
libserial_boost.a: CMakeFiles/serial_boost.dir/build.make
libserial_boost.a: CMakeFiles/serial_boost.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/rover/Boost-Serial-Port-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libserial_boost.a"
	$(CMAKE_COMMAND) -P CMakeFiles/serial_boost.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/serial_boost.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/serial_boost.dir/build: libserial_boost.a
.PHONY : CMakeFiles/serial_boost.dir/build

CMakeFiles/serial_boost.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/serial_boost.dir/cmake_clean.cmake
.PHONY : CMakeFiles/serial_boost.dir/clean

CMakeFiles/serial_boost.dir/depend:
	cd /home/rover/Boost-Serial-Port-master/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rover/Boost-Serial-Port-master /home/rover/Boost-Serial-Port-master /home/rover/Boost-Serial-Port-master/build /home/rover/Boost-Serial-Port-master/build /home/rover/Boost-Serial-Port-master/build/CMakeFiles/serial_boost.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/serial_boost.dir/depend

