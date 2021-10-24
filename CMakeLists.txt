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
)

pico_add_extra_outputs(rebound)

set_target_properties(rebound PROPERTIES COMPILE_FLAGS "-Wall")

pico_set_linker_script(rebound ${CMAKE_CURRENT_SOURCE_DIR}/rebound_noboot2.ld)

target_link_libraries(rebound pico_stdlib_headers pico_stdlib hardware_watchdog hardware_structs hardware_uart hardware_dma pico_sync hardware_pwm)
