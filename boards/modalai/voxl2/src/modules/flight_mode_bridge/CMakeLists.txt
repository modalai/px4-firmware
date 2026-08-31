px4_add_module(
	MODULE modules__flight_mode_bridge
	MAIN flight_mode_bridge
	INCLUDES
		${CMAKE_CURRENT_SOURCE_DIR}/../../lib/mpa
	SRCS
		flight_mode_bridge.cpp
	DEPENDS
		mpa
	)
