#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?, log:" && cat /tmp/log.txt' ERR

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

REPO_PATH=${MY_PATH}/..

## | ------------------------ arguments ----------------------- |

LIST=$1

## | ----------------------------  ---------------------------- |

YAML_FILE=$REPO_PATH/$LIST.yaml

REPOS=$($REPO_PATH/scripts/helpers/parse_yaml_for_docker.py $YAML_FILE)

BUILD_ORDER=""

# clone and checkout
while IFS= read -r REPO; do

  REPOSITORY=$(echo "$REPO" | awk '{print $1}')
  URL=$(echo "$REPO" | awk '{print $2}')
  DOCKER=$(echo "$REPO" | awk '{print $3}')
  AMD=$(echo "$REPO" | awk '{print $4}')
  ARM=$(echo "$REPO" | awk '{print $5}')

  if [[ "$DOCKER" == "False" ]]; then
    continue
  fi

  if [[ $BUILD_ORDER == "" ]]; then
    BUILD_ORDER="[\"$REPOSITORY\""
  else
    BUILD_ORDER="$BUILD_ORDER, \"$REPOSITORY\""
  fi

done < <(echo "$REPOS")

BUILD_ORDER="$BUILD_ORDER]"

echo "" >> /tmp/log.txt 2>&1

echo "$0: ROS package build order:" >> /tmp/log.txt 2>&1
echo "$BUILD_ORDER" >> /tmp/log.txt 2>&1
echo "" >> /tmp/log.txt 2>&1

echo ${BUILD_ORDER}
