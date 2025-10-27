#!/usr/bin/env bash

CONTAINER_NAME=cave_exp

xhost +

sudo docker start $CONTAINER_NAME
sudo docker exec -it $CONTAINER_NAME /bin/bash
