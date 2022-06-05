############################################################################
#
#   Copyright (c) 2015-2017 PX4 Development Team. All rights reserved.
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

px4_add_git_submodule(TARGET git_mavlink_v2 PATH "${PX4_SOURCE_DIR}/mavlink/include/mavlink/v2.0")

if(${PX4_PLATFORM} STREQUAL "qurt")
	set(MAVLINK_FTP_SRC)
	set(MAVLINK_LOG_HANDLER_SRC)
	set(MAVLINK_SIMPLE_ANALYZER_SRC)
	set(MAVLINK_TIMESYNC_SRC)
	set(MAVLINK_TUNE_PUBLISHER_SRC)
	set(MAVLINK_SHELL_SRC)
	set(MAVLINK_ULOG_SRC)
	set(MAVLINK_PARAMETERS_SRC)
else()
	set(MAVLINK_PARAMETERS_SRC mavlink_parameters.cpp)
	set(MAVLINK_FTP_SRC mavlink_ftp.cpp)
	set(MAVLINK_LOG_HANDLER_SRC mavlink_log_handler.cpp)
	set(MAVLINK_SIMPLE_ANALYZER_SRC mavlink_simple_analyzer.cpp)
	set(MAVLINK_TIMESYNC_SRC mavlink_timesync.cpp)
	set(MAVLINK_TUNE_PUBLISHER_SRC tune_publisher.cpp)
	set(MAVLINK_SHELL_SRC mavlink_shell.cpp)
	set(MAVLINK_ULOG_SRC mavlink_ulog.cpp)
endif()

px4_add_module(
	MODULE modules__mavlink
	MAIN mavlink
	COMPILE_FLAGS
		-Wno-cast-align # TODO: fix and enable
		-Wno-address-of-packed-member # TODO: fix in c_library_v2
		-Wno-enum-compare # ROTATION <-> MAV_SENSOR_ROTATION
		#-DDEBUG_BUILD
	INCLUDES
		${PX4_SOURCE_DIR}/mavlink/include/mavlink
	SRCS
		mavlink.c
		mavlink_command_sender.cpp
		mavlink_main.cpp
		mavlink_messages.cpp
		mavlink_mission.cpp
		mavlink_rate_limiter.cpp
		mavlink_receiver.cpp
		mavlink_stream.cpp
		${MAVLINK_PARAMETERS_SRC}
		${MAVLINK_ULOG_SRC}
		${MAVLINK_SHELL_SRC}
		${MAVLINK_FTP_SRC}
		${MAVLINK_TUNE_PUBLISHER_SRC}
		${MAVLINK_LOG_HANDLER_SRC}
		${MAVLINK_SIMPLE_ANALYZER_SRC}
		${MAVLINK_TIMESYNC_SRC}
	MODULE_CONFIG
		module.yaml
	DEPENDS
		airspeed
		drivers_accelerometer
		drivers_barometer
		drivers_gyroscope
		drivers_magnetometer
		git_mavlink_v2
		conversion
		git_ecl
		ecl_geo
		version
	UNITY_BUILD
	)

if(PX4_TESTING)
	add_subdirectory(mavlink_tests)
endif()
