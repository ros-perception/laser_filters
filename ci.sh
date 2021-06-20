#!/bin/bash

# Exit with any error.
set -e

# Should be run from the root directory of the repo.
BUILD_DIR=build

mkdir -p ${BUILD_DIR}
(cd ${BUILD_DIR} && cmake .. -DCATKIN_ENABLE_TESTING=1)

# Build.
make -C ${BUILD_DIR}

# Build the tests.
make -C ${BUILD_DIR} tests

# Run the tests.
make -C ${BUILD_DIR} test

# Summarize test results (also sets the exit status for the script)
catkin_test_results

