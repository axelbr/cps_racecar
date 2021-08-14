.PHONY: build

build: install-deps
	colcon build

install-deps:
	rosdep install --from-paths ./src/* -y