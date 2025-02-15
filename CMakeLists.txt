﻿cmake_minimum_required (VERSION 3.8)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("MIRA")

set(SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/include/Export.h
        ${PROJECT_SOURCE_DIR}/include/MIRAVector.h
        ${PROJECT_SOURCE_DIR}/include/MIRAMatrix.h
        ${PROJECT_SOURCE_DIR}/include/MIRAQuaternion.h
        ${PROJECT_SOURCE_DIR}/include/MIRACollider.h
        ${PROJECT_SOURCE_DIR}/include/MIRARigidBody.h
        ${PROJECT_SOURCE_DIR}/include/CollisionDetection.h
        ${PROJECT_SOURCE_DIR}/include/MIRARigidBody2D.h
        ${PROJECT_SOURCE_DIR}/include/MIRACollider2D.h
        ${PROJECT_SOURCE_DIR}/include/CollisionDetection2D.h
        ${PROJECT_SOURCE_DIR}/include/BroadPhaseGrid.h
        ${PROJECT_SOURCE_DIR}/src/MIRAVector.cpp
        ${PROJECT_SOURCE_DIR}/src/MIRAMatrix.cpp
        ${PROJECT_SOURCE_DIR}/src/MIRAQuaternion.cpp
        ${PROJECT_SOURCE_DIR}/src/MIRACollider.cpp
        ${PROJECT_SOURCE_DIR}/src/MIRARigidBody.cpp
        ${PROJECT_SOURCE_DIR}/src/CollisionDetection.cpp
        ${PROJECT_SOURCE_DIR}/src/MIRARigidBody2D.cpp
        ${PROJECT_SOURCE_DIR}/src/MIRACollider2D.cpp
        ${PROJECT_SOURCE_DIR}/src/CollisionDetection2D.cpp
        ${PROJECT_SOURCE_DIR}/src/BroadPhaseGrid.cpp
)

include_directories(${PROJECT_SOURCE_DIR}/include)

# Create library
add_library(MIRA SHARED ${PROJECT_SOURCE_DIR}/src/main.cpp ${SOURCE_FILES})

target_compile_definitions(MIRA PRIVATE PROJECT_EXPORTS)

############################ Unit Tests ############################
set(SOURCE_FILES_TEST
    ${CMAKE_SOURCE_DIR}/UnitTests/include/MIRATestFramework.h
    ${CMAKE_SOURCE_DIR}/UnitTests/include/MIRABroadPhaseGridTests.h
    ${CMAKE_SOURCE_DIR}/UnitTests/include/MIRACollisionTests.h
    ${CMAKE_SOURCE_DIR}/UnitTests/include/MIRAMathTests.h
    ${CMAKE_SOURCE_DIR}/include/MIRAVector.h
    ${CMAKE_SOURCE_DIR}/include/MIRAMatrix.h
    ${CMAKE_SOURCE_DIR}/include/MIRAQuaternion.h
    ${CMAKE_SOURCE_DIR}/include/MIRACollider.h
    ${CMAKE_SOURCE_DIR}/include/MIRARigidBody.h
    ${CMAKE_SOURCE_DIR}/include/CollisionDetection.h
    ${CMAKE_SOURCE_DIR}/include/MIRARigidBody2D.h
    ${CMAKE_SOURCE_DIR}/include/MIRACollider2D.h
    ${CMAKE_SOURCE_DIR}/include/CollisionDetection2D.h
    ${CMAKE_SOURCE_DIR}/include/BroadPhaseGrid.h
    ${CMAKE_SOURCE_DIR}/src/MIRAVector.cpp
    ${CMAKE_SOURCE_DIR}/src/MIRAMatrix.cpp
    ${CMAKE_SOURCE_DIR}/src/MIRAQuaternion.cpp
    ${CMAKE_SOURCE_DIR}/src/MIRACollider.cpp
    ${CMAKE_SOURCE_DIR}/src/MIRARigidBody.cpp
    ${CMAKE_SOURCE_DIR}/src/CollisionDetection.cpp
    ${CMAKE_SOURCE_DIR}/src/MIRARigidBody2D.cpp
    ${CMAKE_SOURCE_DIR}/src/MIRACollider2D.cpp
    ${CMAKE_SOURCE_DIR}/src/CollisionDetection2D.cpp
    ${CMAKE_SOURCE_DIR}/src/BroadPhaseGrid.cpp
)

include_directories(${CMAKE_SOURCE_DIR}/UnitTests/include)

# Create a test executable
add_executable(MIRA_tests ${CMAKE_SOURCE_DIR}/UnitTests/UnitTestMain.cpp ${SOURCE_FILES_TEST})

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET MIRA PROPERTY CXX_STANDARD 20)
  set_property(TARGET MIRA_tests PROPERTY CXX_STANDARD 20)
endif()

