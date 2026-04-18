libdir = ./lib/
sourcedir = ./src/

UNAME_S := $(shell uname -s)

INCLUDE_DIR = ./include/

ifeq ($(UNAME_S),Linux)
	osdir = linux/
endif
ifeq ($(UNAME_S),Darwin)
	OPENCV_PREFIX = $(shell brew --prefix opencv)
	EIGEN_PREFIX = $(shell brew --prefix eigen)
endif

.PHONY: FORCE clean

main : FORCE $(sourcedir)main.cpp  
	g++ -Wall -Wextra -Wno-deprecated-declarations -O3 -o main $(sourcedir)main.cpp \
	-I$(INCLUDE_DIR)
	-I$(OPENCV_PREFIX)/include/ \
	-I$(EIGEN_PREFIX)/include/ \
    -L$(OPENCV_PREFIX)/lib \
    -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs

clean :
	rm -f main
