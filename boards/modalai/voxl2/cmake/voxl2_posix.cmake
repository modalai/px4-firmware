## Voxl2

include(px4_git)
list(APPEND CMAKE_MODULE_PATH
"${PX4_SOURCE_DIR}/platforms/posix/cmake"
)
set(QC_SOC_TARGET "QRB5165")

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
-D__PX4_POSIX_RB5
-D__PX4_LINUX
)

link_directories(/home ${PX4_SOURCE_DIR}/boards/modalai/voxl2/lib)
