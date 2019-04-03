add_definitions(-std=c++17)   #boost需要17版以上
set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_STANDARD_REQUIRED )
set( CMAKE_CXX_EXTENSIONS OFF )

add_subdirectory(tool)
set(TOOL_INCLUDE_DIR ${TOOL_INCLUDE_DIR} PARENT_SCOPE)
add_subdirectory(velodyne)
set(VELODYNE_INCLUDE_DIR ${VELODYNE_INCLUDE_DIR} PARENT_SCOPE)
add_subdirectory(gui)
set(GUI_INCLUDE_DIR ${GUI_INCLUDE_DIR} PARENT_SCOPE)
add_subdirectory(app)