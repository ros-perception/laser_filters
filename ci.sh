#!/bin/bash

# Exit with any error.
set -e

# Should be run from the root directory of the repo.
BUILD_DIR=build

mkdir -p ${BUILD_DIR}
(cd ${BUILD_DIR} && cmake .. -DCATKIN_ENABLE_TESTING=1)

# Workaround. Invoke cmake twice to ensure that build/catkin_generated/setup_cached.sh adds
# build/devel/lib to LD_LIBRARY_PATH (so that tests use the built laser_filters library instead
# the of installed one).
(cd ${BUILD_DIR} && cmake .. -DCATKIN_ENABLE_TESTING=1)

# Build.
make -C ${BUILD_DIR}

# Build the tests.
make -C ${BUILD_DIR} tests

# Run the tests.
make -C ${BUILD_DIR} test

# Summarize test results (also sets the exit status for the script)
catkin_test_results

