include(Toolchain-hexagon-sdk-8.4)
include(qurt_reqs)

set(module_libraries modules__muorb__slpi)

QURT_LIB(LIB_NAME px4
	# IDL_NAME px4muorb
	SOURCES
		${PX4_SOURCE_DIR}/boards/modalai/voxl2/burner.cpp
	LINK_LIBS
		${module_libraries}
	)

