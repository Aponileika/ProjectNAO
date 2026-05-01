libdir = ../lib/
sourcedir = ./

UNAME_S := $(shell uname -s)

INCLUDE_DIR = ../include/
OPENCVLIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs -lopencv_calib3d -lopencv_videoio -lopencv_features2d

ifeq ($(UNAME_S),Linux)
	osdir = linux/
endif
#This is really just if Jonathans mac... todo
ifeq ($(UNAME_S),Darwin)
	OPENCV = $(shell brew --prefix opencv)
	CERES := $(shell brew --prefix ceres-solver)
	EIGEN := $(shell brew --prefix eigen)
	GLOG := $(shell brew --prefix glog)
	GFLAGS := $(shell brew --prefix gflags)
	SUITESPARSE := $(shell brew --prefix suite-sparse)
	OPENBLAS := $(shell brew --prefix openblas)
	TBB := $(shell brew --prefix tbb)
	
	CXX := clang++
	CXXFLAGS := -Wall -Wextra -Wno-deprecated-declarations -O3 -std=c++17 \
		-DGLOG_USE_GLOG_EXPORT \
		-I$(INCLUDE_DIR) \
		-I$(CERES)/include \
		-I$(EIGEN)/include/eigen3 \
		-I$(SUITESPARSE)/include \
		-I$(OPENCV)/include/opencv4
	
	LDFLAGS := \
	  -L$(CERES)/lib \
	  -L$(GLOG)/lib \
	  -L$(GFLAGS)/lib \
	  -L$(SUITESPARSE)/lib \
	  -L$(OPENBLAS)/lib \
	  -L$(TBB)/lib\
	  -L$(OPENCV)/lib
	
	LDLIBS := \
	  -lceres \
	  -lglog \
	  -lgflags \
	  -lopenblas \
	  -ltbb \
	  $(OPENCVLIBS)

endif

.PHONY: FORCE clean

orbtest: FORCE $(sourcedir)orbtest.cpp  
	$(CXX)  -o orbtest $(sourcedir)orbtest.cpp \
	$(CXXFLAGS) \
	$(LDFLAGS) \
	$(LDLIBS)

clean :
	rm -f orbtest 
