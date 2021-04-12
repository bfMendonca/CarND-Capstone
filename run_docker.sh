#!/bin/bash
docker run -p 4567:4567 -p 11311:11311 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
