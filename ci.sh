#!/bin/bash

# Exit with any error.
set -e

# Should be run from the root directory of the repo.
BUILD_DIR=build

mkdir -p ${BUILD_DIR}
(cd ${BUILD_DIR} && cmake .. -DBUILD_TESTING=1)

# Build.
make -C ${BUILD_DIR}

# Run the tests.
make -C ${BUILD_DIR} test

