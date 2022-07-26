.PHONY: build build-system-image build-host-image run-system-image run-host-image 

build: install-deps
	colcon build

install-deps:
	rosdep install --from-paths ./src/* -y

build-system-image:
	docker build -t cps_racecar/system:latest -f docker/dockerfiles/racecar_system.Dockerfile .

run-system-image:
	docker run --rm -it --gpus all --network host cps_racecar/system:latest
