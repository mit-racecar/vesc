# acceptable build_types: Release/Debug/Profile
build_type=Release
# build_type=Debug

all: build/CMakeLists.txt.copy
	$(info Build_type is [${build_type}])
	$(MAKE) --no-print-directory -C build

clean:
	rm -rf bin lib build msg_gen

build/CMakeLists.txt.copy: build CMakeLists.txt Makefile manifest.xml
	cd build && cmake -DCMAKE_BUILD_TYPE=$(build_type) ..
	cp CMakeLists.txt build/CMakeLists.txt.copy

build:
	mkdir -p build
