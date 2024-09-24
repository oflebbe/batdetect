# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
include(${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
# ====================================================================================
cmake_minimum_required(VERSION 3.13)
# set(PICO_BOARD "sparkfun_thingplus")
# set(PICO_BOARD "pico_w")  # set PICO_W  as well
set(PICO_BOARD "pico2")

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

include(pico_sdk_import.cmake)

project(batdetect C CXX ASM)

pico_sdk_init()



# add_compile_definitions(PICO_W)

add_executable(batdetect batdetect.c st7789.c st7789.h flo_pixmap.h kiss_fft.c kiss_fftr.c)


add_subdirectory(src)
include_directories( src/include)

add_compile_definitions(USE_PRINTF)

pico_generate_pio_header(batdetect ${CMAKE_CURRENT_LIST_DIR}/st7789_parallel.pio)
target_compile_definitions(batdetect PUBLIC PICO_USE_STACK_GUARDS=1 PICO_STACK_SIZE=0x1000)
target_link_libraries(batdetect
	pico_stdlib
	pico_stdio
	hardware_adc
	hardware_dma
	hardware_spi 
	hardware_pio
	hardware_pwm
	hardware_clocks
    pico_time
	pico_util
	pico_multicore
    no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
	)
pico_enable_stdio_usb(batdetect 1)
pico_enable_stdio_uart(batdetect 0)
##
#	pico_define_boot_stage2(slower_boot2 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
#	target_compile_definitions(slower_boot2 PRIVATE PICO_FLASH_SPI_CLKDIV=4)
#	pico_set_boot_stage2( batdetect slower_boot2)

pico_set_program_name(batdetect "batdetect")
pico_set_program_version(batdetect "0.1")

pico_add_extra_outputs(batdetect)
