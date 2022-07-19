
# User Manual

## SDK

The SDK has been tarred and placed in the bucket gs://qrb5165-hexagon-sdk

## ARM tools

The recommended toolchain doesn't support enough c++ for px4:
http://releases.linaro.org/archive/14.11/components/toolchain/binaries/aarch64-linux-gnu/gcc-linaro-4.9-2014.11-x86_64_aarch64-linux-gnu.tar.xz
Use this one instead:
https://releases.linaro.org/components/toolchain/binaries/5.1-2015.08/aarch64-linux-gnu/gcc-linaro-5.1-2015.08-x86_64_aarch64-linux-gnu.tar.xz
TODO: Try a newer one, like https://releases.linaro.org/components/toolchain/binaries/7.5-2019.12/aarch64-linux-gnu/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu.tar.xz

## Environment variables

export ARM_CROSS_GCC_ROOT=/home/ekatzfey/Qualcomm/Hexagon_SDK/4.1.0.4/4.1.0.4/tools/linaro64
export HEXAGON_ARM_SYSROOT=$ARM_CROSS_GCC_ROOT/aarch64-linux-gnu/libc
export HEXAGON_TOOLS_ROOT=$DEFAULT_HEXAGON_TOOLS_ROOT/Tools

## Other

The clean command wipes out both apps and qurt builds
- Use make clean instead of make <target> clean

The clean command runs cmake then deletes all the cmake files???

Why does dspal have the cmake_hexagon directory and the boards/* ???
- Need to switch to a single cmake_hexagon directory

qurt_flags.cmake is used twice. In qurt.cmake and in platforms/qurt/CMakeLists.txt.

## Running

Installing the C++ libraries:
adb push $HEXAGON_SDK_ROOT/tools/HEXAGON_Tools/8.4.05/Tools/target/hexagon/lib/v66/G0/pic/libc++.so.1 /usr/lib/rfsa/adsp
adb push $HEXAGON_SDK_ROOT/tools/HEXAGON_Tools/8.4.05/Tools/target/hexagon/lib/v66/G0/pic/libc++abi.so.1 /usr/lib/rfsa/adsp
adb push $HEXAGON_SDK_ROOT/libs/weak_refs/ship/hexagon_toolv84/weak_refs.so /usr/lib/rfsa/adsp

To allow parameter setting:
mkdir -p /usr/share/data/adsp
mkdir -p /home/linaro/eeprom/parameters

For the data manager (This location can be set with the f flag):
mkdir -p /home/linaro

This is only needed if all of the dsp files are not in "/usr/lib/rfsa/adsp":
export DSP_LIBRARY_PATH="/usr/lib/rfsa/adsp;/usr/lib/rfsa/dsp/sdk"

Default log directory is /home/linaro/log/

```bash
px4 -s /etc/modalai/mainapp.config
```

## SSH
* echo  "PermitRootLogin yes" >> /etc/ssh/sshd_config
* /etc/init.d/ssh restart
* ssh -o IdentitiesOnly=yes root@192.168.0.4
* oelinux123

## SLPI sysmon profiling

### sysMonApp commands on target

* sysMonAppLE_64Bit tlp --q6 sdsp
* sysMonAppLE_64Bit profiler --q6 sdsp

* sysMonAppLE_64Bit getstate --q6 sdsp

* sysMonAppLE_64Bit tinfo --getstack sensors --q6 sdsp

### sysMonParser commands on host for post processing

* $HEXAGON_SDK_ROOT/tools/utils/sysmon/parser_linux_v2/SysmonParser --tlp sysmontlp_sdsp.bin

* $HEXAGON_SDK_ROOT/tools/utils/sysmon/parser_linux_v2/HTML_Parser/sysmon_parser --summary sysmontlp_sdsp.bin
