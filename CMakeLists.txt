cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(rebound C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

#set(PICO_BARE_METAL 1)

pico_sdk_init()

add_executable(rebound
	main.c
	log.c
	comm.c
	util.c
	camera.c
	ov7670.c
)

pico_generate_pio_header(rebound ${CMAKE_CURRENT_LIST_DIR}/camera.pio)

pico_add_extra_outputs(rebound)

set_target_properties(rebound PROPERTIES COMPILE_FLAGS "-Wall")

pico_set_linker_script(rebound ${CMAKE_CURRENT_SOURCE_DIR}/rebound_noboot2.ld)

target_link_libraries(rebound
	pico_multicore
	pico_stdlib
	pico_stdlib_headers
	pico_sync
	pico_util
	hardware_i2c
	hardware_dma
	hardware_pio
	hardware_pwm
	hardware_structs
	hardware_watchdog
	hardware_uart
)
