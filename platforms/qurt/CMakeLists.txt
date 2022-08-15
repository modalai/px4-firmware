
set(module_libraries modules__muorb__slpi)

QURT_LIB(LIB_NAME px4
	SOURCES
		${PX4_SOURCE_DIR}/boards/modalai/voxl2/burner.cpp
	LINK_LIBS
		${module_libraries}
	)
