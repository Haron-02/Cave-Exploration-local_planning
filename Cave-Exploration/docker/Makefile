IMAGE_NAME := ros:cave_exploration
CATKIN_WS_DIR_ := "$(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../../..)"

all: help

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build              - build all images"
# @echo "   1. make pull             - pull all images"
	@echo "   1. make clean              - remove all images"

build:
	@sudo docker build --build-arg CATKIN_WS_DIR=$(CATKIN_WS_DIR_) --tag $(IMAGE_NAME) -f ./Dockerfile ..

clean:
	@sudo docker rmi -f $(IMAGE_NAME)
