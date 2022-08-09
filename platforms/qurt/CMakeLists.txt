include(Toolchain-qurt)
include(qurt_reqs)

set(module_libraries modules__muorb__slpi)

px4_qurt_generate_builtin_commands(
	OUT ${PX4_BINARY_DIR}/apps
	MODULE_LIST ${module_libraries}
)

QURT_LIB(LIB_NAME px4
	# IDL_NAME px4muorb
	SOURCES
		${PX4_BINARY_DIR}/apps.cpp
	LINK_LIBS
		${module_libraries}
	)

