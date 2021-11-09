#!/bin/bash
#
# Run this script from the root of your catkin workspace.
#

# Exit with any error.
set -e

# Build.
catkin_make -DCATKIN_ENABLE_TESTING=1

# Run tests.
catkin_make run_tests

# Summarize test results (also sets the exit status for the script)
catkin_test_results

