# PX4 on VOXL 2

VOXL 2 from ModalAI can be used as a companion computer to a flight control board
but it can also run PX4 directly.

## Overview

When running PX4 directly on the QRB5165 SoC it runs partially on the Sensor Low Power Island (SLPI) DSP (aka sdsp) and partially on the application processor (ARM64 / aarch64).
The portion running on the DSP hosts the flight critical portions of PX4 such as
the IMU, barometer, magnetometer, GPS, ESC, and power management drivers, and the
state estimation. The DSP acts as the real time portion of the system. Non flight
critical applications such as Mavlink, logging, and commander are running on the
ARM CPU cluster (aka apps proc). The DSP and ARM CPU cluster communicate via a
Qualcomm proprietary shared memory interface.

## Build environment

In order to build for this platform both the Qualcomm Hexagon (DSP) toolchain and the Linaro ARM64 toolchain need to be installed. The (nearly) complete setup including the ARM64 toolchain is provided in the base Docker image provided by ModalAI, but since ModalAI is not allowed to redistribute the Qualcomm Hexagon DSP SDK this must be added by the end user.

The full instructions are available here:
- https://gitlab.com/voxl-public/rb5-flight/rb5-flight-px4-build-docker

## Build instructions
