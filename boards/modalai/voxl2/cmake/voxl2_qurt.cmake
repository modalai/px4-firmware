############################################################################
#
# Copyright (c) 2022 ModalAI, Inc. All rights reserved.
#
############################################################################
# VOXL2 is the code name of a board currently in development.
#
# This cmake config builds for QURT which is the operating system running on
# the DSP side.

include(px4_git)
list(APPEND CMAKE_MODULE_PATH
	"${PX4_SOURCE_DIR}/platforms/qurt/cmake"
)

if ("$ENV{HEXAGON_SDK_ROOT}" STREQUAL "")
	message(FATAL_ERROR "Enviroment variable HEXAGON_SDK_ROOT must be set")
else()
	set(HEXAGON_SDK_ROOT $ENV{HEXAGON_SDK_ROOT})
endif()

include(Toolchain-hexagon-sdk-8.4)
include(qurt_reqs)

set(HEXAGON_SDK_INCLUDES ${HEXAGON_SDK_INCLUDES}
	${HEXAGON_SDK_ROOT}/tools/HEXAGON_Tools/8.4.05/Tools/target/hexagon/include
	${PX4_SOURCE_DIR}/platforms/nuttx/Nuttx/nuttx/include
)
include_directories(${HEXAGON_SDK_INCLUDES})

set(CONFIG_SHMEM "0")
add_definitions(-DORB_COMMUNICATOR)
add_definitions(-DRELEASE_BUILD)

set(CONFIG_PARAM_CLIENT "1")

set(DISABLE_PARAMS_MODULE_SCOPING TRUE)
