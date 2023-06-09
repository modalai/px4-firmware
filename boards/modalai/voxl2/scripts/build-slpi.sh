#!/bin/bash

echo "*** Starting qurt slpi build ***"

# source /home/build-env.sh

export HEXAGON_SDK_ROOT=/usr/local/linaro
export DEFAULT_HEXAGON_TOOLS_ROOT=$HEXAGON_SDK_ROOT
export HEXAGON_TOOLS_ROOT=$DEFAULT_HEXAGON_TOOLS_ROOT

make modalai_voxl2-slpi

# cat build/modalai_voxl2-slpi_default/src/lib/version/build_git_version.h

echo "*** End of qurt slpi build ***"
