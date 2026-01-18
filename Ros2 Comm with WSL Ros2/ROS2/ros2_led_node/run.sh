#!/bin/bash

docker run -it --net=host --name ros2_uno_q_dev --rm -v $(pwd)/src:/workspace/src -v /var/run/arduino-router.sock:/var/run/arduino-router.sock ros2-led-ws:latest


