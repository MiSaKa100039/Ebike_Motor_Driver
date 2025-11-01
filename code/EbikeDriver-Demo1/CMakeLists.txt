set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

#set(CMAKE_C_COMPILER_ID GNU)
#set(CMAKE_CXX_COMPILER_ID GNU)

# Some default GCC settings
# arm-none-eabi- must be part of path environment
set(TOOLCHAIN_PREFIX                arm-none-eabi-)

set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wpedantic -fdata-sections -ffunction-sections")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_C_LINK_FLAGS "${TARGET_FLAGS}")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -T${CMAKE_SOURCE_DIR}/STM32G431XX_FLASH.ld")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} --specs=nano.specs")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lc -lm -Wl,--end-group")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--print-memory-usage")

set(CMAKE_CXX_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lstdc++ -lsupc++ -Wl,--end-group")

#////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

cmake_minimum_required(VERSION 3.22)

# ---------------- 项目设置 ----------------
set(CMAKE_PROJECT_NAME EbikeDriver-Demo1)
project(${CMAKE_PROJECT_NAME} C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

message("Build type: " ${CMAKE_BUILD_TYPE})

# ---------------- 工具链 ----------------
#include("cmake/gcc-arm-none-eabi.cmake")

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)   # 生成 compile_commands.json

# ---------------- 预定义宏 ----------------
set(Defines_Syms
        USE_HAL_DRIVER
        STM32G431xx # 具体芯片型号宏
        $<$<CONFIG:Debug>:DEBUG>
)

# ---------------- 头文件路径 ----------------
set(Include_Dirs
        Core/Inc
        Drivers/STM32G4xx_HAL_Driver/Inc
        Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
        Drivers/CMSIS/Device/ST/STM32G4xx/Include
        Drivers/CMSIS/Include

        BSP/communication/VOFA+

        BSP/Motor

        BSP/Signal/LED
        ##################################################

        Platform/communication/VOFA+

        Platform/Motor

        Platform/Signal/LED

        Platform/Utils/Clark
        Platform/Utils/Park
        Platform/Utils/PID
        Platform/Utils/SVPWM

        UserAPP/main
)

# ---------------- 应用层源码 ----------------


file(GLOB_RECURSE Application_Src
        "Core/Src/*.c"
        "BSP/*.c"
        "Platform/*.c/"
        "UserAPP/*.c/"
        "Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/gcc/startup_stm32g431xx.s"
)

# ---------------- HAL 驱动源码 ----------------

file(GLOB_RECURSE STM32_Drivers_Src
        "Drivers/STM32G4xx_HAL_Driver/Src/*.c"
)

# ---------------- HAL 库 ----------------
add_library(STM32_Drivers OBJECT ${STM32_Drivers_Src})
target_include_directories(STM32_Drivers PUBLIC ${Include_Dirs})
target_compile_definitions(STM32_Drivers PUBLIC ${Defines_Syms})

# ---------------- 最终可执行文件 ----------------
add_executable(${CMAKE_PROJECT_NAME} ${Application_Src})
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${Include_Dirs})
target_compile_definitions(${CMAKE_PROJECT_NAME} PUBLIC ${Defines_Syms})

target_link_libraries(${CMAKE_PROJECT_NAME} STM32_Drivers)
target_link_libraries(${PROJECT_NAME} m)

# ---------------- 清理额外文件 ----------------
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

