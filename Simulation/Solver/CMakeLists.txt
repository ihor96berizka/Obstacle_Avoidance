cmake_minimum_required(VERSION 3.20)
project(Solver VERSION 1.0.0 LANGUAGES CXX)

set(DEFAULT_BUILD_TYPE "Release")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

include(GNUInstallDirs)

set(SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/isolver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/solver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/linearsolver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/laplacesolver.cpp
)
set(HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/idataprovider.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/isolver.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/solver.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/linearsolver.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/laplacesolver.h
)

message("----------------")
message(${SOURCE_FILES} ${HEADER_FILES})

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER
    ${CMAKE_CURRENT_SOURCE_DIR}/include/isolver.h)
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER
    ${CMAKE_CURRENT_SOURCE_DIR}/include/solver.h)
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER
    ${CMAKE_CURRENT_SOURCE_DIR}/include/linearsolver.h)
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER
    ${CMAKE_CURRENT_SOURCE_DIR}/include/laplacesolver.h)
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER
    ${CMAKE_CURRENT_SOURCE_DIR}/include/idataprovider.h)


# For access to standard installation directory variables (CMAKE_INSTALL_xDIR).
include(GNUInstallDirs)

# Set library shared object and API header file to install.
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Create the pkg-config file from the template.
configure_file(${PROJECT_NAME}.pc.in ${PROJECT_NAME}.pc @ONLY)

# Set pkg-config file to install.
install(FILES ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
)
