#!/bin/bash

# Run this from the px4 project top level directory
docker run -it --rm -v `pwd`:/usr/local/workspace -v /home/modalai/development/linaro-hexagon/clang+llvm-16.0.0-cross-hexagon-unknown-linux-musl/x86_64-linux-gnu:/usr/local/linaro rb5-flight-px4-build-docker
