############################################################################
# Direct ZVINS2 CDSP -> QSH ADSP odometry ingress.
############################################################################
option(ZQL_DIAGNOSTIC_HOP_TIMING
	"measure CDSP-to-ADSP hop with the intrusive global QTimer"
	OFF)

px4_add_module(
	MODULE drivers__zvins_qsh_link
	MAIN zvins_qsh_link
	STACK_MAIN 4096
	COMPILE_FLAGS
		-fno-fast-math
	SRCS
		zvins_qsh_link.cpp
		zvins_qsh_link_qdi.c
		zvins_qsh_link_wire.c
	)

if(ZQL_DIAGNOSTIC_HOP_TIMING)
	target_compile_definitions(drivers__zvins_qsh_link PRIVATE
		ZQL_DIAGNOSTIC_HOP_TIMING=1)
	message(WARNING
		"ZQL diagnostic hop timing enabled: libpx4 is not a production artifact")
endif()
