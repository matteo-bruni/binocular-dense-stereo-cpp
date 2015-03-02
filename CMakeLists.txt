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

if (NOT CMAKE_BUILD_TYPE)
	message(STATUS "No build type selected, default to Release")
	set(CMAKE_BUILD_TYPE "Release")
endif()

set(SOURCE_FILES
    src/logger/log.h
    src/utils/util.hpp
    src/utils/util.cpp
    src/dataset/dataset.hpp
    src/dataset/msm_middlebury.hpp
    src/dataset/util.hpp
    src/dataset/dataset.cpp
    src/dataset/msm_middlebury.cpp
    src/dataset/util.cpp
    src/test_middlebury.cpp
    src/matching_reproject/stereo_matching.cpp
    src/matching_reproject/stereo_matching.hpp
    src/stereo_viewer/viewer.cpp
    src/stereo_viewer/viewer.hpp
    src/registration/registration.cpp
    src/registration/registration.hpp

)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")


#################################### libconfig  ############################################

#INCLUDE(${CMAKE_SOURCE_DIR}/cmake/FindLibConfig.cmake)
find_package(LibConfig REQUIRED)

#link_directories(${LIBCONFIG_LIBRARIES})
#include_directories(${LIBCONFIG_INCLUDE_DIR})
set(LIBS ${LIBS} ${LIBCONFIGPP_LIBRARY})


#################################### Locate OpenCV ############################################

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
message(${OpenCV_INCLUDE_DIRS})
set(LIBS ${LIBS} ${OpenCV_LIBS})

#################################### Locate PCL ############################################

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#set(LIBS ${LIBS} ${PCL_LIBRARIES})
set(LIBS ${LIBS} ${PCL_COMMON_LIBRARIES})
set(LIBS ${LIBS} ${PCL_IO_LIBRARIES})
set(LIBS ${LIBS} ${PCL_VISUALIZATION_LIBRARIES})
set(LIBS ${LIBS} ${PCL_FILTERS_LIBRARIES})
set(LIBS ${LIBS} ${PCL_FEATURES_LIBRARIES})




message(${LIBS})



#################################### target build ############################################

add_executable(binocular_dense_stereo ${SOURCE_FILES})
target_link_libraries(binocular_dense_stereo ${LIBS})