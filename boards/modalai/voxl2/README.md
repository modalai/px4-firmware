# PX4 on VOXL 2

VOXL 2 from ModalAI can be used as a companion computer to a flight control board
but it can also run PX4 directly.

## Overview

When running PX4 directly on the QRB5165 SoC it runs partially on the Sensor Low Power Island (SLPI) DSP (aka sdsp) and partially on the application processor (ARM64 / aarch64).
The portion running on the DSP hosts the flight critical portions of PX4 such as
the IMU, barometer, magnetometer, GPS, ESC, and power management drivers, and the
state estimation. The DSP acts as the real time portion of the system. Non flight
critical applications such as Mavlink, and logging are running on the
ARM CPU cluster (aka apps proc). The DSP and ARM CPU cluster communicate via a
Qualcomm proprietary shared memory interface.

Both processors are built from this single board directory:
- `default.px4board` - POSIX apps processor (ARM64)
- `slpi.px4board` - QURT DSP (Hexagon)

## Build environment

In order to build for this platform both the Qualcomm Hexagon (DSP) toolchain and the Linaro ARM64 toolchain need to be installed. The (nearly) complete setup including the ARM64 toolchain is provided in the base Docker image provided by ModalAI, but since ModalAI is not allowed to redistribute the Qualcomm Hexagon DSP SDK this must be added by the end user.

The full instructions are available here:
- https://gitlab.com/voxl-public/rb5-flight/rb5-flight-px4-build-docker

## Build overview

A single `make modalai_voxl2` command builds both the DSP and apps processor
firmware. The Makefile chains the SLPI build as a prerequisite of the default
(apps) build.

- Clone the repo (Don't forget to update and initialize all submodules)
- In the top level directory
```
px4$ boards/modalai/voxl2/scripts/run-docker.sh
root@9373fa1401b8:/usr/local/workspace# boards/modalai/voxl2/scripts/clean.sh
root@9373fa1401b8:/usr/local/workspace# boards/modalai/voxl2/scripts/build-apps.sh
root@9373fa1401b8:/usr/local/workspace# exit
```

For DSP-only rebuilds: `make modalai_voxl2_slpi`

## Install and run on VOXL 2

Once the DSP and Linux images have been built they can be installed on a VOXL 2
board using ADB. There is a script to do this.
```
px4$ boards/modalai/voxl2/scripts/install-voxl.sh
```

## Running PX4 on VOXL 2

After installing PX4 on the board, open a terminal on VOXL 2 using ADB shell.
PX4 can be run using a start script.
```
root@m0054:/# voxl-px4
Found DSP signature file
/
INFO  [px4] mlockall() enabled. PX4's virtual address space is locked into RAM.
INFO  [px4] assuming working directory is rootfs, no symlinks needed.

______  __   __    ___
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.

INFO  [px4] Calling startup script: /bin/sh /etc/modalai/voxl-px4.config 0
INFO  [muorb] muorb protobuf initalize method succeeded
INFO  [px4] Startup script returned successfully
pxh>
```

## Vehicle attitude MPA bridge

`vehicle_attitude_bridge` runs on the apps processor and forwards the selected
`vehicle_attitude` uORB topic to the `px4_vehicle_attitude` MPA pipe. It starts
automatically in the normal, SIH, and HITL startup scripts. Its module commands
are `vehicle_attitude_bridge start`, `stop`, and `status` in the PX4 shell (use
the `px4-vehicle_attitude_bridge` executable from a Linux shell).

The shared C/C++ packet definition is provided by libmodal-pipe in
`pipe_interfaces/px4_vehicle_attitude_t.h` and exposed through
`modal_pipe_interfaces.h`. This bridge requires a libmodal-pipe revision that
includes this type. VFC can include the same library header
without PX4 headers. Each 64-byte packet contains a magic number, format version,
publication and sample timestamps in nanoseconds, the current attitude
quaternion, the latest reset quaternion, and the 8-bit reset counter. Both
quaternions use Hamilton `(w, x, y, z)` ordering. The timestamps retain the PX4
topic timebase.

The bridge writes a packet for every attitude update it consumes, including
updates with an unchanged reset counter. It does not throttle the subscription.
Attitude and reset metadata always come from the same uORB sample. Clients
should validate packet size, magic number, and version before using the data.
The library's `pipe_validate_px4_vehicle_attitude_t()` helper checks packet size
and magic numbers; clients must check the version separately.
The header provides a recommended read buffer size that holds multiple packets.

For VFC integration, consume attitude and reset metadata from this pipe together
so that a reset can be handled before using the changed attitude for control.
The bridge alone does not change VFC's control behavior. On initial connection
or reconnection, establish the current attitude/reference and counter as a new
baseline. On a subsequent counter increment, apply the reset once to the stored
attitude reference: `q_reference_new = delta_q_reset * q_reference_old`.
Account for the counter wrapping from 255 to 0. An unchanged counter means the
reset delta must not be applied again.

The topic retains the latest reset delta, not a history of all resets. If a
client misses multiple resets, it must re-establish its reference; the latest
delta is insufficient to reconstruct the missing changes. This stream is also
subject to uORB/MPA delivery gaps and is not a guaranteed reset event log.

## Notes

You cannot cleanly shutdown PX4 with the shutdown command on VOXL 2. You have
to power cycle the board and restart everything. Starting with SDK 1.3.0 it is possible
to cleanly shutdown PX4 on VOXL 2.

## Tips

Always use the latest SDK release

In order to see DSP specific debug messages the mini-dm tool in the Hexagon SDK
can be used (Most messages are passed to the apps proc but certain low level messages are not):
```
modalai@modalai-XPS-15-9570:/local/mnt/workspace/Qualcomm/Hexagon_SDK/4.1.0.4/tools/debug/mini-dm/Ubuntu18$ sudo ./mini-dm
[sudo] password for modalai:
Running mini-dm version: 3.2
Completed processing command line ---
Connecting to the only usbport connected
mini-dm is waiting for a DMSS connection...
DMSS is connected. Running mini-dm...
------------Mini-dm is ready to log-------------
[08500/02]  06:21.030  0069:01: SDSP: Creating new instance.  0355  sns_flight_controller_sensor.
[08500/02]  06:21.030  0069:01: SDSP: Creating data streams  0292  sns_flight_controller_sensor_
[08500/02]  06:21.030  0069:01: SDSP: Configuring devices  0305  sns_flight_controller_sensor_
[08500/02]  06:21.030  0069:01: SDSP: Flight controller sensor instance initialized  0344  sns_flight_controller_sensor_
[08500/01]  06:21.092  0069:01: SDSP: Hello, world!  0000  test
[08500/02]  06:21.092  006a:01: SDSP: CPU Utilization: 0.04, wait percentage 99.94  0162  sns_flight_controller_sensor.
[08500/02]  06:21.092  006a:01: SDSP: Got interrupt registered event  0082  sns_flight_controller_sensor_
```
