cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME EbikeDriver-Demo1)

include(cmake/gcc-arm-none-eabi.cmake)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
# ---------------- 头文件路径 ----------------
set(Include_Dirs

        BSP/Communication/VOFA+

        BSP/Motor

        ##################################################

        Lib/Motor/Motor_Public

        ##################################################

        Platform/Communication/VOFA+

        Platform/Motor

        ##################################################

        UserAPP
)

# ---------------- 应用层源码 ----------------
file(GLOB_RECURSE Application_Src
        "BSP/*.c"
        "Lib/*.c"
        "Platform/*.c"
        "UserAPP/*.c"
)

file(GLOB_RECURSE Application_CXX_Src
        "BSP/*.cpp"
        "Lib/*.cpp"
        "Platform/*.cpp"
        "UserAPP/*.cpp"
)

target_sources(${CMAKE_PROJECT_NAME}
        PRIVATE ${Application_Src}
        PRIVATE ${Application_CXX_Src}
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${Include_Dirs})

# 判断构建类型
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DENABLE_DANGEROUS_TEST_API)
    message(STATUS "Debug Build: Dangerous Test APIs ENABLED")
else()
    message(STATUS "Release Build: Dangerous Test APIs DISABLED")
endif()