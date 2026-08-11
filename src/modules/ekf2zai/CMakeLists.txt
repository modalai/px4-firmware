############################################################################
# Copyright (c) 2026 ModalAI, Inc. All rights reserved.
#
# ekf2zai: EKF2 with the dual-output solution-separation companion.
# Reuses the ekf2 module sources with the class/entry renamed
# (EKF2 -> EKF2ZAI, ekf2_main -> ekf2zai_main); the EKF core symbols are
# provided by the ekf2 module archive (MODULES_EKF2 is a Kconfig dep).
# Run "ekf2zai start" INSTEAD of "ekf2 start" — never both.
############################################################################

px4_add_module(
	MODULE modules__ekf2zai
	MAIN ekf2zai
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
		-fno-associative-math
		-DEKF2=EKF2ZAI
		-Dekf2_main=ekf2zai_main
		-Dekf2_module_mutex=ekf2zai_module_mutex
		-DEKF2_SOLUTION_SEPARATION_MODULE
	INCLUDES
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/../ekf2
		${CMAKE_CURRENT_SOURCE_DIR}/../ekf2/EKF
		${CMAKE_CURRENT_SOURCE_DIR}/../ekf2/EKF/aid_sources
		${CMAKE_CURRENT_SOURCE_DIR}/../ekf2/EKF/python
	PRIORITY
		"SCHED_PRIORITY_MAX - 18"
	STACK_MAX
		3600
	SRCS
		../ekf2/EKF2.cpp
		solution_separation.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		geo
		hysteresis
		perf
		px4_work_queue
		world_magnetic_model
		lat_lon_alt
		bias_estimator
		output_predictor
	)
