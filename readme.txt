1. Download vtk source and compile it using cmake.

2. Enter the vtk build destination and vtk source destination in the CMakeLists.txt as in: 

	set(VTK_DIR "~/Desktop/vtk/build/lib/")				###  BUILD   ###

	set(CMAKE_PREFIX_PATH "~/Downloads/VTK-9.3.0/")     ###  SOURCE  ###

3. Install konsole terminal.

4. Connect the arduino to computer and set the serial port number in arduino_get.cpp

5. Run run.sh

