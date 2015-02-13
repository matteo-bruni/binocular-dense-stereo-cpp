cmake_minimum_required(VERSION 2.8.4)
project(binocular_dense_stereo)


# check c++11 compiler support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# set ouput folder
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

set(SOURCE_FILES
    src/dataset/dataset.hpp
    src/dataset/msm_middlebury.hpp
    src/dataset/util.hpp
    src/dataset/dataset.cpp
    src/dataset/msm_middlebury.cpp
    src/dataset/util.cpp
    src/test_middlebury.cpp
)


#################################### Locate libxml2 ############################################

find_package(OpenCV REQUIRED)
#INCLUDE(${CMAKE_SOURCE_DIR}/cmake/modules/FindLibXML2.cmake)
include_directories( ${OpenCV_INCLUDE_DIRS} )
message(${OpenCV_INCLUDE_DIRS})
set(LIBS ${LIBS} ${OPENCV_INCLUDE_DIR})



add_executable(binocular_dense_stereo ${SOURCE_FILES})
target_link_libraries(binocular_dense_stereo ${OpenCV_LIBS})