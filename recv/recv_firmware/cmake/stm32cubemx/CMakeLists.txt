cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_HAL_DRIVER 
	STM32H743xx
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(stm32cubemx INTERFACE
    ../../Inc
    ../../Drivers/STM32H7xx_HAL_Driver/Inc
    ../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    ../../Middlewares/Third_Party/FreeRTOS/Source/include
    ../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
    ../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    ../../Drivers/CMSIS/Device/ST/STM32H7xx/Include
    ../../Drivers/CMSIS/Include
)

target_sources(stm32cubemx INTERFACE
    ../../Src/main.c
    ../../Src/freertos.c
    ../../Src/usb_device.c
    ../../Src/usbd_conf.c
    ../../Src/usbd_desc.c
    ../../Src/usbd_cdc_if.c
    ../../Src/stm32h7xx_it.c
    ../../Src/stm32h7xx_hal_msp.c
    ../../Src/stm32h7xx_hal_timebase_tim.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd_ex.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c
    ../../Src/system_stm32h7xx.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/list.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/queue.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/timers.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
    ../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    ../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    ../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    ../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
    ../../Src/sysmem.c
    ../../Src/syscalls.c
    ../../startup_stm32h743xx_sramSections.s
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()


