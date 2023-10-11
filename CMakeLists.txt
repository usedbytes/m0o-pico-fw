cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(rebound C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

#set(PICO_BARE_METAL 1)

pico_sdk_init()

add_executable(rebound
	comm.c
	host.c
	input.c
	log.c
	main.c
	util.c

	platform/bno055.c
	platform/boom.c
	platform/chassis.c
	platform/controller.c
	platform/i2c_bus.c
	platform/platform.c
	platform/platform_camera.c
	platform/platform_vl53l0x.c
	platform/kinematics.c
	platform/vl53l0x.c

	plan/apples.c
	plan/plan_chassis.c
	plan/plan_boom.c
	plan/direction.c
	plan/servo.c
	plan/sheep.c
	plan/spin.c
	plan/trough.c
)

target_include_directories(rebound PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
	${CMAKE_CURRENT_LIST_DIR}/platform
	${CMAKE_CURRENT_LIST_DIR}/platform/include
)

add_subdirectory(platform/vl53l0x)
add_subdirectory(platform/camera)
add_subdirectory(platform/ioexpander)

pico_add_extra_outputs(rebound)

set_target_properties(rebound PROPERTIES COMPILE_FLAGS "-Wall")

pico_set_linker_script(rebound ${CMAKE_CURRENT_SOURCE_DIR}/rebound_noboot2.ld)

target_link_libraries(rebound
	camera
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
	ioexpander
	vl53l0x
)
