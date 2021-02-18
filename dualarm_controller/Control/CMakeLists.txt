cmake_minimum_required(VERSION 3.8)
project(Control LANGUAGES CXX)

add_library(${PROJECT_NAME} SHARED STATIC
        Controller.cpp
        KistHand.cpp
        Motion.cpp
        Trajectory.cpp
        )
add_library(sub::CTRL ALIAS ${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})

find_package(Eigen3 3.0 REQUIRED NO_MODULE)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
set(Boost_NO_BOOST_CMAKE ON)
set(BOOST_INCLUDEDIR "/usr/include")
set(BOOST_LIBRARYDIR "/usr/lib/x86_64-linux-gnu")
find_package(Boost COMPONENTS filesystem system QUIET)
if(Boost_FOUND)
    message(STATUS "Boost version: ${Boost_VERSION}")
    INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${Boost_LIBRARIES})
endif()

#find_package(Eigen 3.0.0)
#IF( Eigen_FOUND )
#    add_definitions( ${Eigen_DEFINITIONS} )
#    target_include_directories (${PROJECT_NAME} PUBLIC ${Eigen_INCLUDE_DIRS} )
#ELSE()
#    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN_INCLUDE_DIR to the include directory of your Eigen installation.")
#ENDIF()
