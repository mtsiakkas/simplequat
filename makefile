BUILD_DIR=$(shell pwd)/build/

all: libsimplequat.so

clean:
	@rm -f $(BUILD_DIR)simplequat.o $(BUILD_DIR)libsimplequat.so

simplequat.o:
	g++ -Wall -fPIC -c -std=c++11 simplequat.cpp -o $(BUILD_DIR)simplequat.o

libsimplequat.so: simplequat.o
	g++ -Wall -shared $(BUILD_DIR)simplequat.o -o $(BUILD_DIR)libsimplequat.so
