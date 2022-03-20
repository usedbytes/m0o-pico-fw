add_library(ioexpander INTERFACE)

target_include_directories(ioexpander INTERFACE
	${CMAKE_CURRENT_LIST_DIR}
)

target_sources(ioexpander INTERFACE
	${CMAKE_CURRENT_LIST_DIR}/ioexpander.c
)

target_link_libraries(ioexpander INTERFACE
	pico_time
)
