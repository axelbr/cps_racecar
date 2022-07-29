.PHONY: build sync build-system-image build-system-dev-image run-system run-system-dev

build: install-deps
	colcon build

install-deps:
	rosdep install -y --from-paths src --ignore-src -r -y 

sync:
	rsync -av --filter "merge .rsyncignore" ./ racecar:~/Projects/cps_racecar

build-system-image:
	docker build -t cps_racecar/system:latest -f docker/racecar/racecar.Dockerfile .

build-system-dev-image:
	docker build -t cps_racecar/system:dev -f docker/racecar/racecar.dev.Dockerfile .

run-system:
	docker run --rm -it --gpus all --network host cps_racecar/system:latest

run-system-dev:
	docker run -it \
	--name racecar_dev \
	-v $(CURDIR):/workspace \
	-w /workspace \
	--device /dev/sensors/vesc:/dev/sensors/vesc \
	--network host \
	cps_racecar/system:dev bash