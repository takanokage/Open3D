#!/bin/bash

REAL_PATH=$(dirname $(realpath ${0}))

. ${REAL_PATH}/arguments.sh

echo "stopping containers..."
echo

for ubuntu in ${ubuntu_version[@]}; do
    for bundle in ${bundle_type[@]}; do
        for env in ${env_type[@]}; do
            ${REAL_PATH}/stop.sh ${ubuntu} ${bundle} ${env}
        done
    done
done

echo