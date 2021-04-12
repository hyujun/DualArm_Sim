cmake_minimum_required(VERSION 3.8)
project(Utils LANGUAGES CXX)

set(CMAKE_DEBUG_POSTFIX "d")
file(GLOB SOURCE_FILES "*.cpp" "*.c")

add_library(${PROJECT_NAME} SHARED STATIC
            ${SOURCE_FILES}
            )

add_library(sub::Utils ALIAS ${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
                            PUBLIC ${PRJECT_SOURCE_DIR})

find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
endif(OpenMP_CXX_FOUND)

find_package(Eigen3 3.0 REQUIRED NO_MODULE)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

#find_package(Eigen 3.0.0)
#IF( Eigen_FOUND )
#    target_include_directories (${PROJECT_NAME} PUBLIC ${Eigen_INCLUDE_DIRS} )
#    add_definitions(${Eigen_DEFINITIONS} )
#ELSE()
#    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN_INCLUDE_DIR to the include directory of your Eigen installation.")
#ENDIF()
