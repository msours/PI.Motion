
cmake_minimum_required(VERSION 3.9)

set(project_name PI.Motion)
set(project_version 1.0.0)

set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation Directory")

set(CMAKE_CSharp_FLAGS "/langversion:6")

project(${project_name} LANGUAGES CSharp DESCRIPTION "API for creating fluid motion with an arrangement of PI motion stages and actuators across any number of controllers")

add_library(${project_name} SHARED
    ./source/SingleAxis.cs
    ./source/LinearActuator.cs
    ./source/PI_GCS2.cs
)

set_property(TARGET ${project_name} PROPERTY VS_DOTNET_TARGET_FRAMEWORK_VERSION "v4.0")

set_property(TARGET ${project_name} PROPERTY VS_DOTNET_REFERENCES
    "System"
    "System.Core"
)

#Install

set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation Directory")

file(GLOB depends_dll ./dll/*.dll)

install(TARGETS ${project_name}
 	RUNTIME DESTINATION ${project_name}/bin
)

install (FILES ${depends_dll}
    DESTINATION ${project_name}/bin
)

install(TARGETS ${project_name}
    EXPORT ${PROJECT_NAME}-export
    RUNTIME DESTINATION ${PROJECT_NAME}/bin
)

install(EXPORT ${PROJECT_NAME}-export
    FILE ${PROJECT_NAME}Config.cmake 
    DESTINATION cmake
)