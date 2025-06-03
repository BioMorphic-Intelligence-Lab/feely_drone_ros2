#!/bin/bash
docker buildx build --platform linux/arm64,linux/amd64 -t antbre/tactile_sensor_driver -f ./docker/tactile_sensor_driver.dockerfile --push .