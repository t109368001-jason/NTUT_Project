cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#避免建置路徑與程式碼路徑相同
if(" ${CMAKE_SOURCE_DIR}" STREQUAL " ${CMAKE_BINARY_DIR}")
  message(FATAL_ERROR "
FATAL: In-source builds are not allowed.
       You should create a separate directory for build files.
")
endif()

#清除舊的檔案
set(cmake_generated 
)
foreach(file
        ${CMAKE_BINARY_DIR}/CMakeCache.txt
        ${CMAKE_BINARY_DIR}/cmake_install.cmake  
        ${CMAKE_BINARY_DIR}/Makefile
        ${CMAKE_BINARY_DIR}/CMakeFiles
        ${CMAKE_BINARY_DIR}/bin
    )
    if (EXISTS ${file})
        file(REMOVE_RECURSE ${file})
    endif()

endforeach(file)

#設定變數
set(PROJECT_NAME pcl_viewer)
set(SOURCE_FILE src/main.cpp)
set(SOURCE_FILE src/main.cpp CACHE FILEPATH "SOURCE_FILE")  #cmake -D SOURCE_FILE=xxx ..
set(BUILD_EXAMPLE OFF)
set(BUILD_EXAMPLE OFF CACHE BOOL "BUILD_EXAMPLE")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
    ${CMAKE_BINARY_DIR}/bin
    CACHE PATH
    "Output directory for applications"
    )

project(${PROJECT_NAME})
set(BUILD_REALSENSE OFF)
if(BUILD_REALSENSE)
    set(LIBREALSENSE_INSTALLATION_DIR "" CACHE PATH "librealsense installation directory")
    if(LIBREALSENSE_INSTALLATION_DIR STREQUAL "")
        find_package(realsense2 REQUIRED)
    else()
        find_package(realsense2 REQUIRED PATHS ${LIBREALSENSE_INSTALLATION_DIR} NO_DEFAULT_PATH)
    endif()
endif()

find_package(PCL 1.8 REQUIRED)  #尋找PCL函式庫

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#find_package( OpenCV REQUIRED )
#include_directories( ${OpenCV_INCLUDE_DIRS} )

include(FindPCAP.cmake)
#include(FindZipIos.cmake)

add_definitions(-std=c++17)   #boost需要17版以上

add_executable(${PROJECT_NAME} ${SOURCE_FILE})          #設定執行檔名稱及對應程式碼
if(OpenCV_LIBS)
    target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${PCAP_LIBRARY} ) #連結函式庫
else()
    target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${PCAP_LIBRARY} ) #連結函式庫
endif()

if(BUILD_EXAMPLE)   #設定其他範例執行檔
    add_executable(pcd_viewer ${CMAKE_BINARY_DIR}/../src/pcd_viewer.cpp)
    target_link_libraries(pcd_viewer ${PCL_LIBRARIES})
    add_executable(pcd_poisson_viewer ${CMAKE_BINARY_DIR}/../src/pcd_poisson_viewer.cpp)
    target_link_libraries(pcd_poisson_viewer ${PCL_LIBRARIES})
    add_executable(pcap_viewer ${CMAKE_BINARY_DIR}/../src/pcap_viewer.cpp)
    target_link_libraries(pcap_viewer ${PCL_LIBRARIES})
endif()

#cmake_minimum_required(VERSION 2.8)
## CUDA PACKAGE
#find_package(CUDA REQUIRED)
#set(CUDA_SEPARABLE_COMPILATION ON)
#set(CUDA_PROPAGATE_HOST_FLAGS OFF)
#set(CUDA_HOST_COMPILER clang++)
## COMPILE CU FILES
#file(GLOB CUDA_FILES ${CMAKE_SOURCE_DIR}/include *.cu)
#list( APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_30,code=sm_30; -std=c++17")
#CUDA_COMPILE(CU_O ${CUDA_FILES})
## SETUP FOR CPP FILES
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
## COMPILE AND LINK
#cuda_add_executable(${PROJECT_NAME}_cuda ${SOURCE_FILE} ${CU_O})