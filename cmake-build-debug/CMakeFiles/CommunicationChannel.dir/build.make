# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_COMMAND = /opt/clion-2017.1.3/bin/cmake/bin/cmake

# The command to remove a file.
RM = /opt/clion-2017.1.3/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/andrea/Documents/GIT/CommunicationChannel

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/andrea/Documents/GIT/CommunicationChannel/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/CommunicationChannel.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/CommunicationChannel.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/CommunicationChannel.dir/flags.make

CMakeFiles/CommunicationChannel.dir/CRC_16-CCITT/crc_16ccitt.c.o: CMakeFiles/CommunicationChannel.dir/flags.make
CMakeFiles/CommunicationChannel.dir/CRC_16-CCITT/crc_16ccitt.c.o: ../CRC_16-CCITT/crc_16ccitt.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrea/Documents/GIT/CommunicationChannel/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/CommunicationChannel.dir/CRC_16-CCITT/crc_16ccitt.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/CommunicationChannel.dir/CRC_16-CCITT/crc_16ccitt.c.o   -c /home/andrea/Documents/GIT/CommunicationChannel/CRC_16-CCITT/crc_16ccitt.c

CMakeFiles/CommunicationChannel.dir/CRC_16-CCITT/crc_16ccitt.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/CommunicationChannel.dir/CRC_16-CCITT/crc_16ccitt.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/andrea/Documents/GIT/CommunicationChannel/CRC_16-CCITT/crc_16ccitt.c > CMakeFiles/CommunicationChannel.dir/CRC_16-CCITT/crc_16ccitt.c.i

CMakeFiles/CommunicationChannel.dir/CRC_16-CCITT/crc_16ccitt.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/CommunicationChannel.dir/CRC_16-CCITT/crc_16ccitt.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/andrea/Documents/GIT/CommunicationChannel/CRC_16-CCITT/crc_16ccitt.c -o CMakeFiles/CommunicationChannel.dir/CRC_16-CCITT/crc_16ccitt.c.s

CMakeFiles/CommunicationChannel.dir/CRC_16-CCITT/crc_16ccitt.c.o.requires:

.PHONY : CMakeFiles/CommunicationChannel.dir/CRC_16-CCITT/crc_16ccitt.c.o.requires

CMakeFiles/CommunicationChannel.dir/CRC_16-CCITT/crc_16ccitt.c.o.provides: CMakeFiles/CommunicationChannel.dir/CRC_16-CCITT/crc_16ccitt.c.o.requires
	$(MAKE) -f CMakeFiles/CommunicationChannel.dir/build.make CMakeFiles/CommunicationChannel.dir/CRC_16-CCITT/crc_16ccitt.c.o.provides.build
.PHONY : CMakeFiles/CommunicationChannel.dir/CRC_16-CCITT/crc_16ccitt.c.o.provides

CMakeFiles/CommunicationChannel.dir/CRC_16-CCITT/crc_16ccitt.c.o.provides.build: CMakeFiles/CommunicationChannel.dir/CRC_16-CCITT/crc_16ccitt.c.o


CMakeFiles/CommunicationChannel.dir/CommunicationFunctions/serial_communication_functions.c.o: CMakeFiles/CommunicationChannel.dir/flags.make
CMakeFiles/CommunicationChannel.dir/CommunicationFunctions/serial_communication_functions.c.o: ../CommunicationFunctions/serial_communication_functions.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrea/Documents/GIT/CommunicationChannel/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/CommunicationChannel.dir/CommunicationFunctions/serial_communication_functions.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/CommunicationChannel.dir/CommunicationFunctions/serial_communication_functions.c.o   -c /home/andrea/Documents/GIT/CommunicationChannel/CommunicationFunctions/serial_communication_functions.c

CMakeFiles/CommunicationChannel.dir/CommunicationFunctions/serial_communication_functions.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/CommunicationChannel.dir/CommunicationFunctions/serial_communication_functions.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/andrea/Documents/GIT/CommunicationChannel/CommunicationFunctions/serial_communication_functions.c > CMakeFiles/CommunicationChannel.dir/CommunicationFunctions/serial_communication_functions.c.i

CMakeFiles/CommunicationChannel.dir/CommunicationFunctions/serial_communication_functions.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/CommunicationChannel.dir/CommunicationFunctions/serial_communication_functions.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/andrea/Documents/GIT/CommunicationChannel/CommunicationFunctions/serial_communication_functions.c -o CMakeFiles/CommunicationChannel.dir/CommunicationFunctions/serial_communication_functions.c.s

CMakeFiles/CommunicationChannel.dir/CommunicationFunctions/serial_communication_functions.c.o.requires:

.PHONY : CMakeFiles/CommunicationChannel.dir/CommunicationFunctions/serial_communication_functions.c.o.requires

CMakeFiles/CommunicationChannel.dir/CommunicationFunctions/serial_communication_functions.c.o.provides: CMakeFiles/CommunicationChannel.dir/CommunicationFunctions/serial_communication_functions.c.o.requires
	$(MAKE) -f CMakeFiles/CommunicationChannel.dir/build.make CMakeFiles/CommunicationChannel.dir/CommunicationFunctions/serial_communication_functions.c.o.provides.build
.PHONY : CMakeFiles/CommunicationChannel.dir/CommunicationFunctions/serial_communication_functions.c.o.provides

CMakeFiles/CommunicationChannel.dir/CommunicationFunctions/serial_communication_functions.c.o.provides.build: CMakeFiles/CommunicationChannel.dir/CommunicationFunctions/serial_communication_functions.c.o


CMakeFiles/CommunicationChannel.dir/main.c.o: CMakeFiles/CommunicationChannel.dir/flags.make
CMakeFiles/CommunicationChannel.dir/main.c.o: ../main.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrea/Documents/GIT/CommunicationChannel/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/CommunicationChannel.dir/main.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/CommunicationChannel.dir/main.c.o   -c /home/andrea/Documents/GIT/CommunicationChannel/main.c

CMakeFiles/CommunicationChannel.dir/main.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/CommunicationChannel.dir/main.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/andrea/Documents/GIT/CommunicationChannel/main.c > CMakeFiles/CommunicationChannel.dir/main.c.i

CMakeFiles/CommunicationChannel.dir/main.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/CommunicationChannel.dir/main.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/andrea/Documents/GIT/CommunicationChannel/main.c -o CMakeFiles/CommunicationChannel.dir/main.c.s

CMakeFiles/CommunicationChannel.dir/main.c.o.requires:

.PHONY : CMakeFiles/CommunicationChannel.dir/main.c.o.requires

CMakeFiles/CommunicationChannel.dir/main.c.o.provides: CMakeFiles/CommunicationChannel.dir/main.c.o.requires
	$(MAKE) -f CMakeFiles/CommunicationChannel.dir/build.make CMakeFiles/CommunicationChannel.dir/main.c.o.provides.build
.PHONY : CMakeFiles/CommunicationChannel.dir/main.c.o.provides

CMakeFiles/CommunicationChannel.dir/main.c.o.provides.build: CMakeFiles/CommunicationChannel.dir/main.c.o


# Object files for target CommunicationChannel
CommunicationChannel_OBJECTS = \
"CMakeFiles/CommunicationChannel.dir/CRC_16-CCITT/crc_16ccitt.c.o" \
"CMakeFiles/CommunicationChannel.dir/CommunicationFunctions/serial_communication_functions.c.o" \
"CMakeFiles/CommunicationChannel.dir/main.c.o"

# External object files for target CommunicationChannel
CommunicationChannel_EXTERNAL_OBJECTS =

CommunicationChannel: CMakeFiles/CommunicationChannel.dir/CRC_16-CCITT/crc_16ccitt.c.o
CommunicationChannel: CMakeFiles/CommunicationChannel.dir/CommunicationFunctions/serial_communication_functions.c.o
CommunicationChannel: CMakeFiles/CommunicationChannel.dir/main.c.o
CommunicationChannel: CMakeFiles/CommunicationChannel.dir/build.make
CommunicationChannel: CMakeFiles/CommunicationChannel.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/andrea/Documents/GIT/CommunicationChannel/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking C executable CommunicationChannel"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/CommunicationChannel.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/CommunicationChannel.dir/build: CommunicationChannel

.PHONY : CMakeFiles/CommunicationChannel.dir/build

CMakeFiles/CommunicationChannel.dir/requires: CMakeFiles/CommunicationChannel.dir/CRC_16-CCITT/crc_16ccitt.c.o.requires
CMakeFiles/CommunicationChannel.dir/requires: CMakeFiles/CommunicationChannel.dir/CommunicationFunctions/serial_communication_functions.c.o.requires
CMakeFiles/CommunicationChannel.dir/requires: CMakeFiles/CommunicationChannel.dir/main.c.o.requires

.PHONY : CMakeFiles/CommunicationChannel.dir/requires

CMakeFiles/CommunicationChannel.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/CommunicationChannel.dir/cmake_clean.cmake
.PHONY : CMakeFiles/CommunicationChannel.dir/clean

CMakeFiles/CommunicationChannel.dir/depend:
	cd /home/andrea/Documents/GIT/CommunicationChannel/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrea/Documents/GIT/CommunicationChannel /home/andrea/Documents/GIT/CommunicationChannel /home/andrea/Documents/GIT/CommunicationChannel/cmake-build-debug /home/andrea/Documents/GIT/CommunicationChannel/cmake-build-debug /home/andrea/Documents/GIT/CommunicationChannel/cmake-build-debug/CMakeFiles/CommunicationChannel.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/CommunicationChannel.dir/depend

