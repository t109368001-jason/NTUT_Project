cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

#避免建置路徑與程式碼路徑相同
if(" ${CMAKE_SOURCE_DIR}" STREQUAL " ${CMAKE_BINARY_DIR}")
  message(FATAL_ERROR "
FATAL: In-source builds are not allowed.
       You should create a separate directory for build files.
")
endif()

set(PROJECT_NAME pcl_viewer)

set(CMAKE_AUTOMOC ON)
#set(CMAKE_INCLUDE_CURRENT_DIR ON)

if (UNIX AND NOT APPLE)
    execute_process(COMMAND ${CMAKE_C_COMPILER} -fuse-ld=gold -Wl,--version ERROR_QUIET OUTPUT_VARIABLE ld_version)
    if ("${ld_version}" MATCHES "GNU gold")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fuse-ld=gold -Wl,--disable-new-dtags")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fuse-ld=gold -Wl,--disable-new-dtags")
    endif()
endif()

#設定變數
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
    ${CMAKE_BINARY_DIR}/bin
    CACHE PATH
    "Output directory for applications"
    )

project(${PROJECT_NAME})

#include_directories(${CMAKE_SOURCE_DIR}/velodyne/include)
#add_subdirectory(velodyne)
#set(LIBS ${LIBS} PcapMediaWidget)

#add_library(PcapMediaWidget STATIC IMPORTED)
#set_property(TARGET PcapMediaWidget PROPERTY IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/lib/libPcapMediaWidget.a)
if(NOT PCL_FOUND)
    find_package(PCL 1.8 REQUIRED)  #尋找PCL函式庫
endif()
if(PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    set(LIBS ${LIBS} ${PCL_LIBRARIES})
endif()

if(NOT VTK_FOUND)
    find_package(VTK REQUIRED)
endif()
if(VTK_FOUND)
    include(${VTK_USE_FILE})
    set(LIBS ${LIBS} ${VTK_LIBRARIES})
endif()

if(NOT Qt5Widgets_FOUND)
    find_package(Qt5Widgets CONFIG REQUIRED)
endif()
if(Qt5Widgets_FOUND)
    include_directories(${Qt5Widgets_INCLUDE_DIRS})
    #link_directories(Qt5::Widgets)
    add_definitions(${Qt5Widgets_DEFINITIONS})
endif()

include(FindPCAP)
set(LIBS ${LIBS} ${PCAP_LIBRARY})
set(LIBS ${LIBS} Qt5::Widgets)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty)

add_definitions(-std=c++17)   #boost需要17版以上
set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_STANDARD_REQUIRED )
set( CMAKE_CXX_EXTENSIONS OFF )

add_executable(${PROJECT_NAME} src/main.cpp)          #設定執行檔名稱及對應程式碼
target_link_libraries(${PROJECT_NAME} ${LIBS})
#cotire(${PROJECT_NAME})