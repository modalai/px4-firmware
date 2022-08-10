############################################################################
#
# Copyright (c) 2022 Zach Lowell. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# Overview:
# Hexagon/QuRT apps are built in 2 parts, the part that runs on the
# application (apps) processor, and the library that is invoked on the DSP.
#
# PREREQUISITES:
#
# Environment variables:
#	HEXAGON_TOOLS_ROOT
#	HEXAGON_SDK_ROOT
#
# USAGE:
#
# For simple DSP apps that use a simple apps processor app to invoke the
# DSP lib, the QURT_BUNDLE function can be used.
#
# Build targets to load the apps proc app and libs are created from the
# rules below. Look for resulting make targets ending in -load.

include(px4_git)
list(APPEND CMAKE_MODULE_PATH
"${PX4_SOURCE_DIR}/platforms/posix/cmake"
)
set(DISABLE_PARAMS_MODULE_SCOPING TRUE)

set(CONFIG_SHMEM "0")
add_definitions(-DORB_COMMUNICATOR)
add_definitions(-DRELEASE_BUILD)

set(CONFIG_PARAM_SERVER "1")

add_compile_options($<$<COMPILE_LANGUAGE:C>:-std=gnu99>)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-std=gnu++14>)

add_compile_options(
-Wno-array-bounds
)

add_definitions(
-D__PX4_LINUX
)

link_directories(/home ${PX4_SOURCE_DIR}/boards/modalai/voxl2/lib)

include(CMakeParseArguments)

# Setting hexagon environment up

set(TOOLS_ERROR_MSG
		"HEXAGON_Tools must be installed and the environment variable HEXAGON_TOOLS_ROOT must be set"
		"(e.g. export HEXAGON_TOOLS_ROOT=$ENV{HOME}/Qualcomm/Hexagon_SDK/4.1.0.4/tools)")

if ("$ENV{HEXAGON_TOOLS_ROOT}" STREQUAL "")
	message(FATAL_ERROR ${TOOLS_ERROR_MSG})
else()
	set(HEXAGON_TOOLS_ROOT $ENV{HEXAGON_TOOLS_ROOT})
endif()

if ("$ENV{HEXAGON_SDK_ROOT}" STREQUAL "")
	message(FATAL_ERROR "HEXAGON_SDK_ROOT not set")
endif()

set(SDKINC incs)
set(SDKLIB libs)
set(SDKRPCMEMINC /inc)

set(HEXAGON_SDK_ROOT $ENV{HEXAGON_SDK_ROOT})

set(HEXAGON_SDK_INCLUDES
	${HEXAGON_SDK_ROOT}/${SDKINC}
	${HEXAGON_SDK_ROOT}/${SDKINC}/stddef
	)

# Set the default to SLPI
if ("${DSP_TYPE}" STREQUAL "")
	set(DSP_TYPE "SLPI")
endif()
set(HEXAGON_SDK_INCLUDES ${HEXAGON_SDK_INCLUDES}
	${HEXAGON_SDK_ROOT}/rtos/qurt/computev66/include/qurt
	)

# Validate DSP_TYPE
if (NOT ("${DSP_TYPE}" STREQUAL "ADSP" OR "${DSP_TYPE}" STREQUAL "SLPI"))
	message(FATAL_ERROR "DSP_TYPE set to invalid value")
endif()

# Process DSP files
function (LINUX_LIB)
	set(options)
	set(oneValueArgs LIB_NAME IDL_NAME LIB_DEST)
	set(multiValueArgs SOURCES LINK_LIBS INCS FLAGS)
	cmake_parse_arguments(LINUX_LIB "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

	if ("${LINUX_LIB_IDL_NAME}" STREQUAL "")
		message(FATAL_ERROR "LINUX_LIB called without IDL_NAME")
	endif()

	include_directories(
		${CMAKE_CURRENT_BINARY_DIR}
		)

	add_definitions( "-DDSP_TYPE_${DSP_TYPE}" )

	if (NOT "${LINUX_LIB_SOURCES}" STREQUAL "")

		# Build lib that is run on the DSP
		add_library(${LINUX_LIB_LIB_NAME} SHARED
			${LINUX_LIB_SOURCES}
			${HEXAGON_SDK_ROOT}/${SDKLIB}/common/rpcmem/src/rpcmem.c
			)

		if (NOT "${LINUX_LIB_FLAGS}" STREQUAL "")
			set_target_properties(${LINUX_LIB_LIB_NAME} PROPERTIES COMPILE_FLAGS "${LINUX_LIB_FLAGS}")
		endif()

		if (NOT "${LINUX_LIB_INCS}" STREQUAL "")
			target_include_directories(${LINUX_LIB_LIB_NAME} PUBLIC ${LINUX_LIB_INCS})
		endif()

		target_link_libraries(${LINUX_LIB_LIB_NAME}
			${LINUX_LIB_LINK_LIBS}
			)
	endif()
endfunction()

# Process Apps proc app source and libs
function (LINUX_APP)
	set(oneValueArgs APP_NAME IDL_NAME APP_DEST)
	set(multiValueArgs SOURCES LINK_LIBS INCS)
	cmake_parse_arguments(LINUX_APP "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

	if ("${LINUX_APP_SOURCES}" STREQUAL "")
		message(FATAL_ERROR "LINUX_APP called without SOURCES")
	endif()

	if ("${LINUX_APP_IDL_NAME}" STREQUAL "")
		message(FATAL_ERROR "LINUX_APP called without IDL_NAME")
	endif()

	include_directories(
		${CMAKE_CURRENT_BINARY_DIR}
		)

	add_definitions( "-DDSP_TYPE_${DSP_TYPE}" )

	# Build lib that is run on the DSP
	add_executable(${LINUX_APP_APP_NAME}
		${LINUX_APP_SOURCES}
		)

	if (NOT "${LINUX_APP_INCS}" STREQUAL "")
		target_include_directories(${LINUX_APP_APP_NAME} PUBLIC ${LINUX_APP_INCS})
	endif()

	target_link_libraries(${LINUX_APP_APP_NAME}
		${LINUX_APP_LINK_LIBS}
		)
endfunction()

