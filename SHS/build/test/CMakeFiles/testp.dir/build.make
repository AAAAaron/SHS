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
CMAKE_SOURCE_DIR = /home/aaron/projects/SHS

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/aaron/projects/SHS/build

# Include any dependencies generated for this target.
include test/CMakeFiles/testp.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/testp.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/testp.dir/flags.make

test/CMakeFiles/testp.dir/test_attitude.cpp.o: test/CMakeFiles/testp.dir/flags.make
test/CMakeFiles/testp.dir/test_attitude.cpp.o: ../test/test_attitude.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aaron/projects/SHS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/testp.dir/test_attitude.cpp.o"
	cd /home/aaron/projects/SHS/build/test && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testp.dir/test_attitude.cpp.o -c /home/aaron/projects/SHS/test/test_attitude.cpp

test/CMakeFiles/testp.dir/test_attitude.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testp.dir/test_attitude.cpp.i"
	cd /home/aaron/projects/SHS/build/test && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aaron/projects/SHS/test/test_attitude.cpp > CMakeFiles/testp.dir/test_attitude.cpp.i

test/CMakeFiles/testp.dir/test_attitude.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testp.dir/test_attitude.cpp.s"
	cd /home/aaron/projects/SHS/build/test && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aaron/projects/SHS/test/test_attitude.cpp -o CMakeFiles/testp.dir/test_attitude.cpp.s

test/CMakeFiles/testp.dir/test_attitude.cpp.o.requires:

.PHONY : test/CMakeFiles/testp.dir/test_attitude.cpp.o.requires

test/CMakeFiles/testp.dir/test_attitude.cpp.o.provides: test/CMakeFiles/testp.dir/test_attitude.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/testp.dir/build.make test/CMakeFiles/testp.dir/test_attitude.cpp.o.provides.build
.PHONY : test/CMakeFiles/testp.dir/test_attitude.cpp.o.provides

test/CMakeFiles/testp.dir/test_attitude.cpp.o.provides.build: test/CMakeFiles/testp.dir/test_attitude.cpp.o


# Object files for target testp
testp_OBJECTS = \
"CMakeFiles/testp.dir/test_attitude.cpp.o"

# External object files for target testp
testp_EXTERNAL_OBJECTS =

../bin/testp: test/CMakeFiles/testp.dir/test_attitude.cpp.o
../bin/testp: test/CMakeFiles/testp.dir/build.make
../bin/testp: ../lib/libSHS.so
../bin/testp: /usr/local/lib/libopencv_shape.so.3.4.3
../bin/testp: /usr/local/lib/libopencv_ml.so.3.4.3
../bin/testp: /usr/local/lib/libopencv_stitching.so.3.4.3
../bin/testp: /usr/local/lib/libopencv_superres.so.3.4.3
../bin/testp: /usr/local/lib/libopencv_videostab.so.3.4.3
../bin/testp: /usr/local/lib/libopencv_calib3d.so.3.4.3
../bin/testp: /usr/local/lib/libopencv_features2d.so.3.4.3
../bin/testp: /usr/local/lib/libopencv_highgui.so.3.4.3
../bin/testp: /usr/local/lib/libopencv_videoio.so.3.4.3
../bin/testp: /usr/local/lib/libopencv_video.so.3.4.3
../bin/testp: /usr/local/lib/libopencv_dnn.so.3.4.3
../bin/testp: /usr/local/lib/libopencv_flann.so.3.4.3
../bin/testp: /usr/local/lib/libopencv_photo.so.3.4.3
../bin/testp: /usr/local/lib/libopencv_imgcodecs.so.3.4.3
../bin/testp: /usr/local/lib/libopencv_viz.so.3.4.3
../bin/testp: /usr/local/lib/libopencv_objdetect.so.3.4.3
../bin/testp: /usr/local/lib/libopencv_imgproc.so.3.4.3
../bin/testp: /usr/local/lib/libopencv_core.so.3.4.3
../bin/testp: test/CMakeFiles/testp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aaron/projects/SHS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/testp"
	cd /home/aaron/projects/SHS/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/testp.dir/build: ../bin/testp

.PHONY : test/CMakeFiles/testp.dir/build

test/CMakeFiles/testp.dir/requires: test/CMakeFiles/testp.dir/test_attitude.cpp.o.requires

.PHONY : test/CMakeFiles/testp.dir/requires

test/CMakeFiles/testp.dir/clean:
	cd /home/aaron/projects/SHS/build/test && $(CMAKE_COMMAND) -P CMakeFiles/testp.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/testp.dir/clean

test/CMakeFiles/testp.dir/depend:
	cd /home/aaron/projects/SHS/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aaron/projects/SHS /home/aaron/projects/SHS/test /home/aaron/projects/SHS/build /home/aaron/projects/SHS/build/test /home/aaron/projects/SHS/build/test/CMakeFiles/testp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/testp.dir/depend
