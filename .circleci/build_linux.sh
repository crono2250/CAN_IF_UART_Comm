#!/bin/bash

TARGETS=(Release Debug)

for target in ${TARGETS[@]}; do
    cd $target
    cmake ..
    cmake --build .
    cd ..
done
