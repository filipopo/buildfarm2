#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

REPO_PATH=$MY_PATH/..

ARTIFACT_FOLDER=$1

[ -z $ARTIFACT_FOLDER ] && ARTIFACT_FOLDER=/tmp/artifacts
[ -z $RUN_LOCALLY ] && RUN_LOCALLY=false

WORKSPACE=/tmp/workspace

echo "$0: extracting workspace"

# extract the workspace
if ! $RUN_LOCALLY; then
  cd $ARTIFACT_FOLDER
  tar -xvzf workspace.tar.gz -C /tmp/
fi

echo "$0: workspace extracted"

ls -la $WORKSPACE/src

# are there any coverage files?

$REPO_PATH/ci_scripts/helpers/wait_for_docker.sh

docker pull klaxalk/lcov

ARGS=""

for file in `ls $ARTIFACT_FOLDER | grep ".info"`; do

  if [ -s ${ARTIFACT_FOLDER}/${file} ]; then
    ARGS="${ARGS} -a ${ARTIFACT_FOLDER}/${file}"
  fi

done

echo "$0: combining coverage"

docker run \
  --rm \
  -v $WORKSPACE:/tmp/workspace \
  -v $ARTIFACT_FOLDER:/tmp/artifacts \
  klaxalk/lcov \
  /bin/bash -c "lcov $ARGS --output-file /tmp/workspace/coverage_temp.info"

echo "$0: filtering coverage"

docker run \
  --rm \
  -v $WORKSPACE:/tmp/workspace \
  klaxalk/lcov \
  /bin/bash -c "lcov --remove /tmp/workspace/coverage_temp.info --ignore-errors unused,unused '*/eth_*' --output-file /tmp/workspace/coverage.info || echo 'coverage tracefile is empty'"

echo "$0: generating coverage html"

docker run \
  --rm \
  -v $WORKSPACE:/tmp/workspace \
  klaxalk/lcov \
  /bin/bash -c "genhtml --title 'MRS UAV System - Test coverage report' --prefix '/tmp/workspace/src' --demangle-cpp --legend --frames --show-details -o /tmp/workspace/coverage_html /tmp/workspace/coverage.info | tee /tmp/workspace/coverage.log"


COVERAGE_PCT=`cat $WORKSPACE/coverage.log | grep -E "lines\.\.\." | awk '{print $2}'`

echo "Coverage: $COVERAGE_PCT"

docker pull klaxalk/pybadges

docker run \
  --rm \
  -v $WORKSPACE:/tmp/workspace \
  klaxalk/pybadges \
  /bin/bash -c "/bin/python3 -m pybadges --left-text='test coverage' --right-text='${COVERAGE_PCT}' --right-color='#0c0' > /tmp/workspace/coverage_html/badge.svg"
