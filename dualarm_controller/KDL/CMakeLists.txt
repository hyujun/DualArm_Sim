cmake_minimum_required(VERSION 3.8)
project(KDL LANGUAGES CXX)

add_library(${PROJECT_NAME} SHARED STATIC
        LieDynamics.cpp
        LieOperator.cpp
        PoEKinematics.cpp
        PropertyDefinition.cpp
        SerialManipulator.cpp
        )

add_library(sub::KDL ALIAS ${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
        PUBLIC ${PROJECT_SOURCE_DIR})

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
