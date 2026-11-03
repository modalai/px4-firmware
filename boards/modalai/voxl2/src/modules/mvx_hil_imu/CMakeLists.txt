############################################################################
#
#   Copyright (c) 2026 ModalAI, inc. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE modules__mvx_hil_imu
	MAIN mvx_hil_imu
	SRCS
		mvx_hil_imu.cpp
	DEPENDS
		drivers_accelerometer
		drivers_gyroscope
	)
