sourcedir = ./MIT/src/

UNAME_S := $(shell uname -s)

INCLUDE_DIR = ./MIT/include/
EXT_INCLUDE_DIR = /usr/local/include
EXT_LIB_DIR = /usr/local/lib

OPENCVLIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs -lopencv_calib3d -lopencv_videoio -lopencv_features2d

OPENCV = $(shell brew --prefix opencv)
CERES := $(shell brew --prefix ceres-solver)
EIGEN := $(shell brew --prefix eigen)
GLOG := $(shell brew --prefix glog)
GFLAGS := $(shell brew --prefix gflags)
SUITESPARSE := $(shell brew --prefix suite-sparse)
OPENBLAS := $(shell brew --prefix openblas)
TBB := $(shell brew --prefix tbb)
SOPHUS := $(shell brew --prefix sophus)

CXX := clang++

BASE_CXXFLAGS := -Wall -Wextra -Wno-deprecated-declarations -std=c++20 \
	-DGLOG_USE_GLOG_EXPORT \
	-I$(INCLUDE_DIR) \
	-I$(CERES)/include \
	-I$(EIGEN)/include/eigen3 \
	-I$(SUITESPARSE)/include \
	-I$(EXT_INCLUDE_DIR) \
	-I$(OPENCV)/include/opencv4 \
	-I$(SOPHUS)/include 

ifeq ($(DEBUG),1)
	CXXFLAGS := $(BASE_CXXFLAGS) -O0 -g -fsanitize=address -fno-omit-frame-pointer -DDEBUG
	SANITIZER_LDFLAGS := -fsanitize=address
else ifeq($(CONFIG_IMU, 1))
	CXXFLAGS := $(BASE_CXXFLAGS) -O2 -DCONFIG_IMU
	SANITIZER_LDFLAGS :=
else
	CXXFLAGS := $(BASE_CXXFLAGS) -O2
	SANITIZER_LDFLAGS :=
endif

LDFLAGS := \
  -L$(CERES)/lib \
  -L$(GLOG)/lib \
  -L$(GFLAGS)/lib \
  -L$(SUITESPARSE)/lib \
  -L$(OPENBLAS)/lib \
  -L$(TBB)/lib \
  -L$(EXT_LIB_DIR) \
  -L$(OPENCV)/lib \
  -Wl,-rpath,$(EXT_LIB_DIR) \
  $(SANITIZER_LDFLAGS)

LDLIBS := \
  -lceres \
  -lglog \
  -lgflags \
  -lopenblas \
  -ltbb \
  -lDBoW3 \
  $(OPENCVLIBS)


.PHONY: FORCE clean imu_test

main : FORCE $(sourcedir)main.cpp
	$(CXX) -o main $(sourcedir)main.cpp \
	$(CXXFLAGS) \
	$(LDFLAGS) \
	$(LDLIBS) \
	-ferror-limit=100

imu_test : MIT/test/IMU_Test.cpp
	$(CXX) -o imu_test MIT/test/IMU_Test.cpp \
	-DPANTO_ACTIVE_DATASET=EUROC_MAV_VICON_ROOM1_EASY \
	$(CXXFLAGS) \
	$(LDFLAGS) \
	$(LDLIBS) \
	-ferror-limit=100

clean :
	rm -f main imu_test
