BUILD_DIR=$(shell pwd)/build/

all:
	g++ -Wall -c -fPIC -std=c++11 simplequat.cpp -o $(BUILD_DIR)simplequat.o
	g++ -Wall -shared $(BUILD_DIR)simplequat.o -o $(BUILD_DIR)libsimplequat.so

clean:
	@rm -f $(BUILD_DIR)simplequat.o $(BUILD_DIR)libsimplequat.so

simplequat.o:
	g++ -Wall -c -std=c++11 simplequat.cpp -o $(BUILD_DIR)simplequat.o

libsimplequat.so: simplequat.o
	g++ -Wall -shared $(BUILD_DIR)simplequat.o -o $(BUILD_DIR)libsimplequat.so
