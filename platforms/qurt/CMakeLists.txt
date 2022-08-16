
set(module_libraries modules__muorb__slpi)

QURT_LIB(LIB_NAME px4
	SOURCES
		${PX4_SOURCE_DIR}/platforms/qurt/unresolved_symbols.c
	LINK_LIBS
		${module_libraries}
	)
