#!/bin/bash
set -e

echo "Building base image..."
docker build -t ros2-base:jazzy -f docker/Dockerfile.base .

echo "Building merged workspace image..."
docker build -t ros2-led-ws:latest -f docker/Dockerfile.led_ws .

echo "Build complete."

