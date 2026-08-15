UNAME_S := $(shell uname -s)

INCLUDE_DIR = ../include/
EXT_INCLUDE_DIR = /usr/local/include
EXT_LIB_DIR = /usr/local/lib

OPENCVLIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs -lopencv_calib3d -lopencv_videoio -lopencv_features2d

OPENCV := $(shell brew --prefix opencv)
EIGEN := $(shell brew --prefix eigen)

CXX := clang++
CXXFLAGS := -Wall -Wextra -Wno-deprecated-declarations -O3 -std=c++20 \
	-DGLOG_USE_GLOG_EXPORT \
	-I$(INCLUDE_DIR) \
	-I$(EIGEN)/include/eigen3 \
	-I$(EXT_INCLUDE_DIR) \
	-I$(OPENCV)/include/opencv4

LDFLAGS := \
  -L$(OPENCV)/lib \
  -L$(EXT_LIB_DIR)

LDLIBS := \
  -lDBoW3 \
  $(OPENCVLIBS)


.PHONY: FORCE clean

main : FORCE DBOW3_BuildVocab.cpp  
	$(CXX)  -o main DBOW3_BuildVocab.cpp \
	$(CXXFLAGS) \
	$(LDFLAGS) \
	$(LDLIBS) \
	-ferror-limit=100

clean :
	rm -f main
