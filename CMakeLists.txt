cmake_minimum_required(VERSION 3.13)
#set(PICO_BOARD "sparkfun_thingplus")
# set(PICO_BOARD "pico_w")  # set PICO_W  as well
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

include(pico_sdk_import.cmake)

project(adc_fft_project C CXX ASM)

pico_sdk_init()

add_compile_options("-Wall" )

# add_compile_definitions(PICO_W)

add_executable(adc_fft adc_fft.c st7789.c st7789_font.c kiss_fft.c kiss_fftr.c)

pico_enable_stdio_usb(adc_fft 0)
pico_enable_stdio_uart(adc_fft 0)

add_subdirectory(src)
include_directories( src/include)

# add_compile_definitions(USE_PRINTF)

pico_generate_pio_header(adc_fft ${CMAKE_CURRENT_LIST_DIR}/st7789_parallel.pio)

target_link_libraries(adc_fft
	pico_stdlib
	pico_stdio
	hardware_adc
	hardware_dma
	hardware_spi 
	hardware_pio
	hardware_pwm
# 	pico_cyw43_arch_none
    pico_time
	pico_util
	pico_multicore
    no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
	)

	#pico_define_boot_stage2(slower_boot2 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
	#target_compile_definitions(slower_boot2 PRIVATE PICO_FLASH_SPI_CLKDIV=4)
	#pico_set_boot_stage2( adc_fft slower_boot2)

pico_set_program_name(adc_fft "batdetect")
pico_set_program_version(adc_fft "0.1")

pico_add_extra_outputs(adc_fft)
