#!/bin/bash

adb push ../../../build/modalai_rb5-flight_qurt/platforms/qurt/libpx4.so /usr/lib/rfsa/adsp

adb push ../../../build/modalai_rb5-flight_default/bin/px4 /usr/bin
adb push ../../../build/modalai_rb5-flight_default/bin/px4-alias.sh /usr/bin
adb shell chmod a+x /usr/bin/px4-alias.sh
adb shell mkdir -p /etc/modalai
adb push voxl-px4.config /etc/modalai
adb push voxl-px4-set-default-parameters.config /etc/modalai
adb push voxl-px4 /usr/bin
adb shell chmod +x /usr/bin/voxl-px4

adb shell sync
