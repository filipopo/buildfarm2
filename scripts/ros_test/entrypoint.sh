#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

REPOSITORY_NAME=$1

WORKSPACE=/tmp/workspace
COVERAGE=/etc/docker/coverage
COREDUMP=/etc/docker/coredump

echo "$0: installing dependencies using rosdep"

rosdep install -y -v --from-path $WORKSPACE/src

## | ---------------- initialize the workspace ---------------- |

if [ ! -e $WORKSPACE/install ]; then

  echo "$0: workspace not initialized, initializing"

  cd $WORKSPACE

  source /opt/ros/jazzy/setup.bash
  colcon build --cmake-args -DCOVERAGE=true -DMRS_ENABLE_TESTING=true

fi

## | ------------------- build the workspace ------------------ |

echo "$0: building the workspace"

cd $WORKSPACE

colcon build --cmake-args -DCOVERAGE=true -DMRS_ENABLE_TESTING=true

source $WORKSPACE/install/setup.bash

## | --- run tests an all ros packages within the repository -- |

cd $WORKSPACE

FAILED=0

colcon test --base-paths $WORKSPACE/src/$REPOSITORY_NAME || FAILED=1

echo "$0: tests finished"

## | ---------------------- save coverage --------------------- |

if [[ "$FAILED" -eq 0 ]]; then

  echo "$0: storing coverage data"

  # gather all the coverage data from the workspace
  lcov --capture --directory ${WORKSPACE} --output-file /tmp/coverage.original --ignore-errors mismatch,mismatch,gcov,gcov,negative,negative

  echo "$0: filtering out tests"

  # filter out unwanted files, i.e., test files
  lcov --remove /tmp/coverage.original "*/test/*" --output-file /tmp/coverage.removed || echo "$0: coverage tracefile is empty"

  echo "$0: extract sources"

  # extract coverage data for the source folder of the workspace
  lcov --extract /tmp/coverage.removed "$WORKSPACE/src/*" --output-file $COVERAGE/$REPOSITORY_NAME.info || echo "$0: coverage tracefile is empty"

fi

exit $FAILED
