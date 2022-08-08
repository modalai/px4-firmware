############################################################################
#
# Copyright (c) 2015 Mark Charlebois. All rights reserved.
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
# Hexagon SDK paths need to be set based on env variables
#
# PREREQUISITES:
#
# Environment variables:
#	HEXAGON_TOOLS_ROOT
#	HEXAGON_SDK_ROOT
#
# CMake Variables:
#	QC_SOC_TARGET
#
# OPTIONAL:
#	DSP_TYPE (ADSP or SLPI)

# message(FATAL_ERROR "ERIC hexagon_sdk.cmake")

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

if ("${QC_SOC_TARGET}" STREQUAL "QRB5165")
	# Set the default to SLPI
	if ("${DSP_TYPE}" STREQUAL "")
		set(DSP_TYPE "SLPI")
	endif()
	set(V_ARCH "v66")
	set(HEXAGON_SDK_INCLUDES ${HEXAGON_SDK_INCLUDES}
		${HEXAGON_SDK_ROOT}/rtos/qurt/computev66/include/qurt
		)
else()
	message(FATAL_ERROR "QC_SOC_TARGET not set")
endif()

# Validate DSP_TYPE
if (NOT ("${DSP_TYPE}" STREQUAL "ADSP" OR "${DSP_TYPE}" STREQUAL "SLPI"))
	message(FATAL_ERROR "DSP_TYPE set to invalid value")
endif()

############################################################################
#
# Copyright (c) 2015 Mark Charlebois. All rights reserved.
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

#include(hexagon_sdk)

# message(FATAL_ERROR "ERIC fastrpc.cmake")

if("${RELEASE}" STREQUAL "")
	set(RELEASE Debug)
endif()

if(NOT ("${RELEASE}" STREQUAL "Debug" OR "${RELEASE}" STREQUAL "Release"))
	message(FATAL "RELEASE must be set to Debug or Release")
endif()

set(FASTRPC_DSP_INCLUDES
	${HEXAGON_SDK_INCLUDES}
	${HEXAGON_SDK_ROOT}/${SDKLIB}/common/rpcmem
	${HEXAGON_SDK_ROOT}/${SDKLIB}/common/remote/ship/hexagon_${RELEASE}
	)

set(FASTRPC_ARM_LINUX_INCLUDES
	${HEXAGON_SDK_INCLUDES}
	${HEXAGON_SDK_ROOT}/${SDKLIB}/common/rpcmem
	${HEXAGON_SDK_ROOT}/${SDKLIB}/common/adspmsgd/ship/UbuntuARM_${RELEASE}
	${HEXAGON_SDK_ROOT}/${SDKLIB}/common/remote/ship/UbuntuARM_${RELEASE}
	)

if ("${DSP_TYPE}" STREQUAL "ADSP")
	set(ADSPRPC -L${HEXAGON_SDK_ROOT}/${SDKLIB}/common/remote/ship/UbuntuARM_${RELEASE} -ladsprpc)
elseif("${DSP_TYPE}" STREQUAL "SLPI")
	set(ADSPRPC -L${HEXAGON_SDK_ROOT}/ipc/fastrpc/remote/ship/UbuntuARM_aarch64 -lsdsprpc)
else()
	message(FATAL_ERROR "DSP_TYPE not defined")
endif()

set(ADSPMSGD ${HEXAGON_SDK_ROOT}/${SDKLIB}/common/adspmsgd/ship/UbuntuARM_${RELEASE}/adspmsgd.a)

set(FASTRPC_ARM_LIBS
	${ADSPRPC}
	)


include_directories(
	${CMAKE_CURRENT_BINARY_DIR}
	)

function(FASTRPC_STUB_GEN IDLFILE)
	get_filename_component(FASTRPC_IDL_NAME ${IDLFILE} NAME_WE)
	get_filename_component(FASTRPC_IDL_PATH ${IDLFILE} ABSOLUTE)
	set (IDLINCS ${ARGN})

	# prepend -I in front of QAIC include dirs
	set(QAIC_INCLUDE_DIRS)
	foreach(inc ${IDLINCS})
		string(SUBSTRING ${inc} 0 1 absolute_path_character)
		if (absolute_path_character STREQUAL "/")
			list(APPEND QAIC_INCLUDE_DIRS -I${inc})
			message("QAIC include directory: -I${inc}")
		else()
			list(APPEND QAIC_INCLUDE_DIRS -I${CMAKE_CURRENT_SOURCE_DIR}/${inc})
			message("QAIC include directory: -I${CMAKE_CURRENT_SOURCE_DIR}/${inc}")
		endif()
	endforeach()

	# Run the IDL compiler to generate the stubs
	add_custom_command(
		OUTPUT ${FASTRPC_IDL_NAME}.h ${FASTRPC_IDL_NAME}_skel.c ${FASTRPC_IDL_NAME}_stub.c
		DEPENDS ${FASTRPC_IDL_PATH}
		COMMAND "${HEXAGON_SDK_ROOT}/ipc/fastrpc/qaic/bin/qaic" "-mdll" "-I" "${HEXAGON_SDK_ROOT}/${SDKINC}/stddef" ${QAIC_INCLUDE_DIRS} ${FASTRPC_IDL_PATH}
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		)

	message("Generated generate_${FASTRPC_IDL_NAME}_stubs target")

	add_custom_target(generate_${FASTRPC_IDL_NAME}_stubs ALL
		DEPENDS ${FASTRPC_IDL_NAME}.h ${FASTRPC_IDL_NAME}_skel.c ${FASTRPC_IDL_NAME}_stub.c
		)

	set_source_files_properties(
		${FASTRPC_IDL_NAME}.h
		${FASTRPC_IDL_NAME}_skel.c
		${FASTRPC_IDL_NAME}_stub.c
		PROPERTIES
		GENERATED TRUE
		)
endfunction()

############################################################################
#
# Copyright (c) 2015 PX4 Development Team. All rights reserved.
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

#=============================================================================
#
#	Defined functions in this file
#
# 	OS Specific Functions
#
#		* px4_qurt_add_firmware
#		* px4_qurt_generate_builtin_commands
#		* px4_qurt_add_export
#		* px4_qurt_generate_romfs
#
# 	Required OS Inteface Functions
#
# 		* px4_os_add_flags
# 		* px4_os_determine_build_chip
#		* px4_os_prebuild_targets
#

#=============================================================================
#
#	px4_qurt_generate_builtin_commands
#
#	This function generates the builtin_commands.c src for qurt
#
#	Usage:
#		px4_qurt_generate_builtin_commands(
#			MODULE_LIST <in-list>
#			OUT <file>)
#
#	Input:
#		MODULE_LIST	: list of modules
#
#	Output:
#		OUT	: stem of generated apps.cpp/apps.h ("apps").
#
#	Example:
#		px4_qurt_generate_builtin_commands(
#			OUT <generated-src> MODULE_LIST px4_simple_app)
#
function(px4_qurt_generate_builtin_commands)
	px4_parse_function_args(
		NAME px4_qurt_generate_builtin_commands
		ONE_VALUE OUT
		MULTI_VALUE MODULE_LIST
		REQUIRED MODULE_LIST OUT
		ARGN ${ARGN})

	set(builtin_apps_string)
	set(builtin_apps_decl_string)
	set(command_count 0)
	foreach(module ${MODULE_LIST})
		foreach(property MAIN STACK_MAIN PRIORITY)
			get_target_property(${property} ${module} ${property})
		endforeach()
		if (MAIN)
			set(builtin_apps_string
				"${builtin_apps_string}\tapps[\"${MAIN}\"] = ${MAIN}_main;\n")
			set(builtin_apps_decl_string
				"${builtin_apps_decl_string}int ${MAIN}_main(int argc, char *argv[]);\n")
			math(EXPR command_count "${command_count}+1")
		endif()
	endforeach()
	configure_file(${PX4_SOURCE_DIR}/platforms/common/apps.cpp.in ${OUT}.cpp)
	configure_file(${PX4_SOURCE_DIR}/platforms/common/apps.h.in ${OUT}.h)
endfunction()

#=============================================================================
#
#	px4_os_add_flags
#
#	Set the qurt build flags.
#
#	Usage:
#		px4_os_add_flags()
#
function(px4_os_add_flags)

	set(DSPAL_ROOT platforms/qurt/dspal)
	include_directories(
		${DSPAL_ROOT}/include
		${DSPAL_ROOT}/sys
		${DSPAL_ROOT}/sys/sys

		platforms/posix/include
		platforms/qurt/include
	)

	add_definitions(
		-D__PX4_POSIX
		-D__PX4_QURT
	)

	add_compile_options(
		-fPIC
		-fmath-errno

		-Wno-unknown-warning-option
		-Wno-cast-align
	)

	# Clear -rdynamic flag which fails for hexagon
	set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS)
	set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS)

endfunction()

#=============================================================================
#
#	px4_os_determine_build_chip
#
#	Sets PX4_CHIP and PX4_CHIP_MANUFACTURER.
#
#	Usage:
#		px4_os_determine_build_chip()
#
function(px4_os_determine_build_chip)

	# always use generic chip and chip manufacturer
	set(PX4_CHIP "generic" CACHE STRING "PX4 Chip" FORCE)
	set(PX4_CHIP_MANUFACTURER "generic" CACHE STRING "PX4 Chip Manufacturer" FORCE)

endfunction()

#=============================================================================
#
#	px4_os_prebuild_targets
#
#	This function generates os dependent targets
#
#	Usage:
#		px4_os_prebuild_targets(
#			OUT <out-list_of_targets>
#			BOARD <in-string>
#			)
#
#	Input:
#		BOARD		: board
#
#	Output:
#		OUT	: the target list
#
#	Example:
#		px4_os_prebuild_targets(OUT target_list BOARD px4_fmu-v2)
#
function(px4_os_prebuild_targets)
	px4_parse_function_args(
			NAME px4_os_prebuild_targets
			ONE_VALUE OUT BOARD
			REQUIRED OUT
			ARGN ${ARGN})

	add_library(prebuild_targets INTERFACE)
	add_dependencies(prebuild_targets DEPENDS uorb_headers)

endfunction()

#
# Copyright (C) 2016 Mark Charlebois. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#	notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#	notice, this list of conditions and the following disclaimer in
#	the documentation and/or other materials provided with the
#	distribution.
# 3. Neither the name ATLFlight nor the names of its contributors may be
#	used to endorse or promote products derived from this software
#	without specific prior written permission.
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

message(STATUS "*** Entering qurt_flags.cmake ***")

#include (hexagon_sdk)

set(TOOLSLIB ${HEXAGON_TOOLS_ROOT}/target/hexagon/lib/${V_ARCH}/G0/pic)
set(HEXAGON_ARCH_FLAGS
	-march=hexagon
	-mcpu=hexagon${V_ARCH}
	)

macro (list2string out in)
	set(list ${ARGV})
	list(REMOVE_ITEM list ${out})
	foreach(item ${list})
		set(${out} "${${out}} ${item}")
	endforeach()
endmacro(list2string)

set(HEXAGON_START_LINK_FLAGS)
list2string(HEXAGON_START_LINK_FLAGS
	${HEXAGON_ARCH_FLAGS}
	-shared
	-call_shared
	-G0
	${TOOLSLIB}/initS.o
	"-o <TARGET>"
	-L${TOOLSLIB}
	-Bsymbolic
	${TOOLSLIB}/libgcc.a
	--wrap=malloc
	--wrap=calloc
	--wrap=free
	--wrap=realloc
	--wrap=memalign
	--wrap=__stack_chk_fail
	-lc
	"-soname=<TARGET_SONAME>"
	)

set(HEXAGON_END_LINK_FLAGS)
list2string(HEXAGON_END_LINK_FLAGS
	--start-group
	-lgcc
	--end-group
	${TOOLSLIB}/finiS.o
	)

set(CMAKE_C_CREATE_SHARED_LIBRARY
	"${HEXAGON_LINK} ${HEXAGON_START_LINK_FLAGS} --start-group --whole-archive <OBJECTS> <LINK_LIBRARIES> --end-group ${HEXAGON_END_LINK_FLAGS}")

# TODO: Revisit the linking in of C++ libraries
set(CMAKE_CXX_CREATE_SHARED_LIBRARY
	"${HEXAGON_LINK} ${HEXAGON_START_LINK_FLAGS} --start-group ${TOOLSLIB}/libc++.so.1 ${TOOLSLIB}/libc++abi.so.1 ${HEXAGON_SDK_ROOT}/libs/weak_refs/ship/hexagon_toolv84/weak_refs.so --end-group --start-group --whole-archive <OBJECTS> <LINK_LIBRARIES> --end-group ${HEXAGON_END_LINK_FLAGS}")
#	"${HEXAGON_LINK} ${HEXAGON_START_LINK_FLAGS} --start-group --whole-archive <OBJECTS> <LINK_LIBRARIES> --no-whole-archive ${TOOLSLIB}/libc++.a --end-group ${HEXAGON_END_LINK_FLAGS}")

set(DYNAMIC_LIBS -Wl,${TOOLSLIB}/libc++.a)

#set(MAXOPTIMIZATION -O0)

# Base CPU flags for each of the supported architectures.
#
set(ARCHCPUFLAGS
	-m${V_ARCH}
	-G0
	-DDSP_TYPE_${DSP_TYPE}
	)

add_definitions(
	-D __QURT
	-D _PID_T
	-D _UID_T
	-D _TIMER_T
	-D _HAS_C9X
	-D restrict=__restrict__
	-D noreturn_function=
	)

# Language-specific flags
#
set(ARCHCFLAGS
	-D__CUSTOM_FILE_IO__
	)

set(ARCHCXXFLAGS
	-DCONFIG_WCHAR_BUILTIN
	-D__CUSTOM_FILE_IO__
	)

exec_program(${CMAKE_CXX_COMPILER} ${CMAKE_CURRENT_SOURCE_DIR} ARGS -print-libgcc-file-name OUTPUT_VARIABLE LIBGCC)
exec_program(${CMAKE_CXX_COMPILER} ${CMAKE_CURRENT_SOURCE_DIR} ARGS -print-file-name=libm.a OUTPUT_VARIABLE LIBM)
set(EXTRA_LIBS ${EXTRA_LIBS} ${LIBM})

# Flags we pass to the C compiler
#
list2string(CFLAGS
	${ARCHCFLAGS}
	${ARCHCPUFLAGS}
	${HEXAGON_INCLUDE_DIRS}
	)

# Flags we pass to the C++ compiler
#
list2string(CXXFLAGS
    -Wno-inconsistent-missing-override
	${ARCHCXXFLAGS}
	${ARCHCPUFLAGS}
	${HEXAGON_INCLUDE_DIRS}
	)

# Flags we pass to the assembler
#
list2string(AFLAGS
	${CFLAGS}
	-D__ASSEMBLY__
	)

# Set cmake flags
#
list2string(QURT_CMAKE_C_FLAGS
	${CMAKE_C_FLAGS}
	${CFLAGS}
	)

set(CMAKE_C_FLAGS "${QURT_CMAKE_C_FLAGS}")


list2string(QURT_CMAKE_CXX_FLAGS
	${CMAKE_CXX_FLAGS}
	${CXXFLAGS}
	)

set(CMAKE_CXX_FLAGS ${QURT_CMAKE_CXX_FLAGS})

# Flags we pass to the linker
# CMake make test builds of apps to validate the compiler
# we never use a linked app for the Hexagon as apps are run as
# dynamic libraries and invoked via FastRPC
# These settings enable CMake to build the required test apps
list2string(CMAKE_EXE_LINKER_FLAGS
	-m${V_ARCH}
	-mG0lib
	-G0
	-fpic
	-shared
	-Wl,-Bsymbolic
	-Wl,--wrap=malloc
	-Wl,--wrap=calloc
	-Wl,--wrap=free
	-Wl,--wrap=realloc
	-Wl,--wrap=memalign
	-Wl,--wrap=__stack_chk_fail
	${DYNAMIC_LIBS}
	-lc
	${EXTRALDFLAGS}
	)

message(STATUS "CMAKE_C_FLAGS: ${CMAKE_C_FLAGS}")
message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

message(STATUS "*** Exiting qurt_flags.cmake ***")

############################################################################
#
# Copyright (c) 2015 Mark Charlebois. All rights reserved.
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
# When the apps proc app requires its own cmake build, the RPC stub functions
# can be generated with FASTRPC_STUB_GEN. The DSP lib can be built with the
# QURT_LIB function and the apps proc app can be built with the help of the
# the FASTRPC_ARM_APP_DEPS_GEN function.
#
# Build targets to load the apps proc app and DSP libs are created from the
# rules below. Look for resulting make targets ending in -load.

# message(FATAL_ERROR "ERIC qurt_lib.cmake")

#include(fastrpc)

include (CMakeParseArguments)

# Process DSP files
function (QURT_LIB)
	set(options)
	set(oneValueArgs LIB_NAME IDL_NAME)
	set(multiValueArgs SOURCES LINK_LIBS INCS FLAGS)
	cmake_parse_arguments(QURT_LIB "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

	#if ("${QURT_LIB_IDL_NAME}" STREQUAL "")
	#	message(FATAL_ERROR "QURT_LIB called without IDL_NAME")
	#endif()

	include_directories(
		${CMAKE_CURRENT_BINARY_DIR}
		${FASTRPC_DSP_INCLUDES}
		)
	message("QURT_LIB_INCS = ${QURT_LIB_INCS}")

	#add_library(${QURT_LIB_IDL_NAME}_skel MODULE
	#	${QURT_LIB_IDL_NAME}_skel.c
	#	)

	if ("${QURT_LIB_SOURCES}" STREQUAL "")
		message(FATAL_ERROR "QURT_LIB called without SOURCES")
	else()
		# Build lib that is run on the DSP
		add_library(${QURT_LIB_LIB_NAME} SHARED
			${QURT_LIB_SOURCES}
			)

		if (NOT "${QURT_LIB_FLAGS}" STREQUAL "")
			set_target_properties(${QURT_LIB_LIB_NAME} PROPERTIES COMPILE_FLAGS "${QURT_LIB_FLAGS}")
		endif()

		if (NOT "${QURT_LIB_INCS}" STREQUAL "")
			target_include_directories(${QURT_LIB_LIB_NAME} PUBLIC ${QURT_LIB_INCS})
		endif()

		message("QURT_LIB_LINK_LIBS = ${QURT_LIB_LINK_LIBS}")

		target_link_libraries(${QURT_LIB_LIB_NAME}
			${QURT_LIB_LINK_LIBS}
			)

		# add_dependencies(${QURT_LIB_LIB_NAME} generate_${QURT_LIB_IDL_NAME}_stubs)

		# Hack to support PX4 - because it links static libs into .so targets it ends up with
		# Duplicate symbols. This can be reverted to link ${QURT_LIB_LIB_NAME} when PX4 is fixed.
		#target_link_libraries(${QURT_LIB_IDL_NAME}_skel
		#	"${CMAKE_CURRENT_BINARY_DIR}/lib${QURT_LIB_LIB_NAME}.so"
		#	)

		#add_dependencies(${QURT_LIB_IDL_NAME}_skel
		#	generate_${QURT_LIB_IDL_NAME}_stubs
		#	${QURT_LIB_LIB_NAME}
		#	)
	endif()

	#message("Making custom target build_${QURT_LIB_LIB_NAME}_dsp")
	#add_custom_target(build_${QURT_LIB_LIB_NAME}_dsp ALL
	#	DEPENDS ${QURT_LIB_IDL_NAME} ${QURT_LIB_IDL_NAME}_skel
	#	)

   if ("${QC_SOC_TARGET}" STREQUAL "APQ8096")
      # Set the location for 8x96 target
      set(DSPLIB_TARGET_PATH "/usr/lib/rfsa/adsp/")
   else()
      # 8x74 target is assumed by default.
      set(DSPLIB_TARGET_PATH "/usr/share/data/adsp/")
   endif()

	if ("${QURT_LIB_LIB_NAME}" STREQUAL "")
		# Add a rule to load the files onto the target that run in the DSP
		add_custom_target(lib${QURT_LIB_IDL_NAME}_skel-load
			DEPENDS ${QURT_LIB_IDL_NAME}_skel
			COMMAND adb wait-for-device
			COMMAND adb push lib${QURT_LIB_IDL_NAME}_skel.so ${DSPLIB_TARGET_PATH}
			COMMAND echo "Pushed lib${QURT_LIB_IDL_NAME}_skel.so ${DSPLIB_TARGET_PATH}"
			)
	else()
		# Add a rule to load the files onto the target that run in the DSP
		add_custom_target(lib${QURT_LIB_LIB_NAME}-load
			DEPENDS ${QURT_LIB_LIB_NAME} ${QURT_LIB_IDL_NAME}_skel
			COMMAND adb wait-for-device
			COMMAND adb push lib${QURT_LIB_IDL_NAME}_skel.so ${DSPLIB_TARGET_PATH}
			COMMAND adb push lib${QURT_LIB_LIB_NAME}.so ${DSPLIB_TARGET_PATH}
			COMMAND echo "Pushed lib${QURT_LIB_LIB_NAME}.so and dependencies to ${DSPLIB_TARGET_PATH}"
			)
	endif()
endfunction()
