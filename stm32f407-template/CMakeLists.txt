cmake_minimum_required(VERSION 3.20)
project(stm32f407_baremetal LANGUAGES C ASM)

set(MCU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS "${MCU_FLAGS} -DSTM32F407xx -O2 -g3 -Wall -Wextra -ffunction-sections -fdata-sections")
set(CMAKE_ASM_FLAGS "${MCU_FLAGS} -g3")

set(SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/src/main.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/src/system_stm32f4xx.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/startup_stm32f407xx.s"
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/CORE/inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/BSP/inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Core/Include"
    # "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include"
)

configure_file(STM32F407VGTx_FLASH.ld STM32F407VGTx_FLASH.ld COPYONLY)

target_link_options(${PROJECT_NAME} PRIVATE
    -TSTM32F407VGTx_FLASH.ld
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    -nostartfiles -nostdlib -nodefaultlibs
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
)