cmake_minimum_required(VERSION 3.12)
#set(PICO_BOARD "sparkfun_thingplus")
set(PICO_BOARD "pico_w")


include(pico_sdk_import.cmake)

project(adc_fft_project)

pico_sdk_init()

add_compile_options("-Wall" "-fanalyzer")

add_executable(adc_fft adc_fft.c st7789.c st7789_font.c kiss_fft.c kiss_fftr.c)

pico_enable_stdio_usb(adc_fft 1)
pico_enable_stdio_uart(adc_fft 1)


pico_generate_pio_header(adc_fft ${CMAKE_CURRENT_LIST_DIR}/st7789_parallel.pio)

target_link_libraries(adc_fft
	pico_stdlib
	pico_stdio
	hardware_adc
	hardware_dma
	hardware_spi 
	hardware_pio
	hardware_pwm
	pico_cyw43_arch_none
	)

pico_add_extra_outputs(adc_fft)
