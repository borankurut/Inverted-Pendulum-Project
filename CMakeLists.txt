cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(Play)

set(VTK_DIR "~/Desktop/vtk/build/lib/")
set(CMAKE_PREFIX_PATH "~/Downloads/VTK-9.3.0/")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(VTK COMPONENTS 
  CommonColor
  CommonCore
  CommonDataModel
  CommonTransforms
  FiltersGeneral
  FiltersModeling
  FiltersSources
  InteractionStyle
  RenderingContextOpenGL2
  RenderingCore
  RenderingFreeType
  RenderingGL2PSOpenGL2
  RenderingOpenGL2
  CommonCore
  InteractionStyle
  RenderingContextOpenGL2
  RenderingCore
  RenderingFreeType
  RenderingGL2PSOpenGL2
  RenderingOpenGL2
  CommonColor
  CommonCore
  CommonTransforms
  FiltersSources
  InteractionStyle
  InteractionWidgets
  RenderingContextOpenGL2
  RenderingCore
  RenderingFreeType
  RenderingGL2PSOpenGL2
  RenderingOpenGL2
)

if (NOT VTK_FOUND)
  message(FATAL_ERROR "Play: Unable to find the VTK build folder.")
endif()

# Prevent a "command line is too long" failure in Windows.
set(CMAKE_NINJA_FORCE_RESPONSE_FILE "ON" CACHE BOOL "Force Ninja to use response files.")
add_executable(Play MACOSX_BUNDLE play.cpp)
  target_link_libraries(Play PRIVATE ${VTK_LIBRARIES}
)

# vtk_module_autoinit is needed
vtk_module_autoinit(
  TARGETS Play
  MODULES ${VTK_LIBRARIES}
)

