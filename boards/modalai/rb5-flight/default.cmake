
# rb5 flight

include(px4_git)
# px4_add_git_submodule(TARGET git_cmake_hexagon PATH "${PX4_SOURCE_DIR}/boards/modalai/cmake_hexagon")
list(APPEND CMAKE_MODULE_PATH
	"${PX4_SOURCE_DIR}/boards/modalai/cmake_hexagon"
	"${PX4_SOURCE_DIR}/boards/modalai/cmake_hexagon/toolchain"
	)

set(QC_SOC_TARGET "QRB5165")

# Disable the creation of the parameters.xml file by scanning individual
# source files, and scan all source files.  This will create a parameters.xml
# file that contains all possible parameters, even if the associated module
# is not used.  This is necessary for parameter synchronization between the
# ARM and DSP processors.
set(DISABLE_PARAMS_MODULE_SCOPING TRUE)

set(CONFIG_SHMEM "0")
add_definitions(-DORB_COMMUNICATOR)
# add_definitions(-DDEBUG_BUILD)
add_definitions(-DRELEASE_BUILD)

set(CONFIG_PARAM_SERVER "1")

# modalai toolchain doesn't properly set the compiler, so these aren't set automatically
add_compile_options($<$<COMPILE_LANGUAGE:C>:-std=gnu99>)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-std=gnu++14>)

add_compile_options(
    -Wno-array-bounds
)

add_definitions(
	-D__PX4_POSIX_RB5
	-D__PX4_LINUX
    -DCONFIG_BOARDCTL_RESET
)

link_directories(/home ${PX4_SOURCE_DIR}/boards/modalai/rb5-flight/lib)

px4_add_board(
	PLATFORM posix
	VENDOR modalai
	MODEL rb5-flight
	LABEL default
	#TESTING
	TOOLCHAIN aarch64-linux-gnu
	DRIVERS
		#barometer # all available barometer drivers
		#batt_smbus
		#camera_trigger
		#differential_pressure # all available differential pressure drivers
		#distance_sensor # all available distance sensor drivers
		gps
		#imu # all available imu drivers
		#lights/rgbled
		#magnetometer # all available magnetometer drivers
		#pwm_out_sim
		px4io
		qshell/posix
		#rc_input
        modalai/imu_server
        #modalai/rc_controller
        #modalai/uart_esc_driver
		#telemetry # all available telemetry drivers
        spektrum_rc
	MODULES
		#airspeed_selector
		#attitude_estimator_q
		#camera_feedback
		commander
		dataman
		modalai_gps_timer
		#ekf2
		#events
        flight_mode_manager
		#fw_att_control
		#fw_pos_control_l1
		#land_detector
		#landing_target_estimator
		#load_mon
		#local_position_estimator
		logger
		mavlink
		#mc_att_control
		#mc_pos_control
		#mc_rate_control
		#micrortps_bridge
		muorb/apps
		#muorb/test
		navigator
		rc_update
		#rover_pos_control
		#sensors
		#sih
		simulator
		#vmount
		#vtol_att_control
	SYSTEMCMDS
		#bl_update
		#config
		#dumpfile
		esc_calib
		#hardfault_log
		led_control
		mixer
		motor_ramp
		motor_test
		#mtd
		#nshterm
		param
		perf
		pwm
		reboot
		sd_bench
		shutdown
		#tests # tests and test runner
		#top
		topic_listener
		tune_control
		ver
		work_queue
	EXAMPLES
		#bottle_drop # OBC challenge
		#fixedwing_control # Tutorial code from https://px4.io/dev/example_fixedwing_control
		#hello
		#hwtest # Hardware test
		#matlab_csv_serial
		#px4_mavlink_debug # Tutorial code from http://dev.px4.io/en/debug/debug_values.html
		#px4_simple_app # Tutorial code from http://dev.px4.io/en/apps/hello_sky.html
		#rover_steering_control # Rover example app
		#uuv_example_app
	)
