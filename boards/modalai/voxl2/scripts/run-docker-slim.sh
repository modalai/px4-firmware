#!/bin/bash

# Run this from the px4 project top level directory
# docker run -it --rm -v `pwd`:/usr/local/workspace rb5-flight-px4-build-docker
docker run -it --rm \
    -v `pwd`:/usr/local/workspace \
    -v /home/modalai/development/px4/slim-hexagon-sdk:/usr/local/slim \
    -v /home/modalai/development/linaro-hexagon:/usr/local/linaro \
    rb5-flight-px4-build-docker
