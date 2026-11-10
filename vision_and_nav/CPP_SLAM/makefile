sourcedir = ./MIT/src/

UNAME_S := $(shell uname -s)

INCLUDE_DIR = ./MIT/include/
EXT_INCLUDE_DIR = /usr/local/include
EXT_LIB_DIR = /usr/local/lib

OPENCVLIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs -lopencv_calib3d -lopencv_videoio -lopencv_features2d -lopencv_xfeatures2d

OPENCV = $(shell brew --prefix opencv)
# OPENCV = /Users/Jonathan/Programmering/FIA/PANTOPILOT/vision_and_nav/CPP_SLAM/vendor/opencv/build
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
	-I$(OPENBLAS)/include \
	-I$(EIGEN)/include/eigen3 \
	-I$(SUITESPARSE)/include \
	-I$(EXT_INCLUDE_DIR) \
	-I$(OPENCV)/include/opencv4 \
	-I$(SOPHUS)/include 

ifeq ($(UNAME_S),Darwin)
LIBOMP := $(shell brew --prefix libomp)
BASE_CXXFLAGS += -I$(LIBOMP)/include
endif

#CXXFLAGS := $(BASE_CXXFLAGS) -O3 -DNDEBUG -march=native -mcpu=native -ffp-contract=fast -flto
CXXFLAGS := $(BASE_CXXFLAGS) -O3 
SANITIZER_LDFLAGS :=

ifeq ($(DEBUG),1)
	CXXFLAGS := $(BASE_CXXFLAGS) -O0 -g \
		-fsanitize=address \
		-fno-omit-frame-pointer \
		-DDEBUG
	SANITIZER_LDFLAGS := -fsanitize=address
endif

ifeq ($(CONFIG_IMU),1)
	CXXFLAGS += -DCONFIG_IMU
endif

ifeq ($(CONFIG_STEREO),1)
	CXXFLAGS += -DCONFIG_STEREO
endif

ifeq ($(CONFIG_PLANAR),1)
	CXXFLAGS += -DCONFIG_PLANAR
endif

ifneq ($(origin USE_GT_INIT),undefined)
CXXFLAGS += -DPANTO_GROUNDTRUTH_INIT=$(USE_GT_INIT)
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
  $(SANITIZER_LDFLAGS)

ifeq ($(UNAME_S),Darwin)
LDFLAGS += -L$(LIBOMP)/lib -Wl,-rpath,$(EXT_LIB_DIR)
OP_TEST_LDFLAGS := -Wl,-dead_strip
else
OP_TEST_LDFLAGS := -Wl,--gc-sections
endif

LDLIBS := \
  -lceres \
  -lglog \
  -lgflags \
  -lopenblas \
  -ltbb \
  -lDBoW3 \
  $(OPENCVLIBS)

ifeq ($(UNAME_S),Darwin)
LDLIBS += -lomp -framework Accelerate
endif


.PHONY: FORCE clean imu_test op_step3_test no_gt_init_test viz_export_test gt_alignment_test threading_test map_lock_test descriptor_cache_test tracking_snapshot_test

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

op_step3_test : MIT/test/OP_Step3Test.cpp
	$(CXX) -o op_step3_test MIT/test/OP_Step3Test.cpp \
	-DCONFIG_IMU -DCONFIG_STEREO \
	$(CXXFLAGS) -ffunction-sections -fdata-sections \
	$(LDFLAGS) $(OP_TEST_LDFLAGS) \
	$(LDLIBS) \
	-ferror-limit=100
	./op_step3_test

no_gt_init_test : MIT/test/SL_NoGTInitTest.cpp
	$(CXX) -o no_gt_init_test MIT/test/SL_NoGTInitTest.cpp \
	$(BASE_CXXFLAGS) -O2 -DDEBUG -DCONFIG_IMU -DCONFIG_STEREO \
	-DPANTO_GROUNDTRUTH_INIT=0 -DPANTO_DATASET_REALTIME_MODE=0 \
	-ffunction-sections -fdata-sections \
	$(LDFLAGS) $(OP_TEST_LDFLAGS) $(LDLIBS)
	./no_gt_init_test

viz_export_test : MIT/test/VIZ_ExportTest.cpp
	$(CXX) -o viz_export_test MIT/test/VIZ_ExportTest.cpp \
	-DCONFIG_IMU -DCONFIG_STEREO $(CXXFLAGS) -ffunction-sections -fdata-sections \
	$(LDFLAGS) $(OP_TEST_LDFLAGS) $(LDLIBS)
	./viz_export_test

gt_alignment_test : MIT/test/GT_AlignmentTest.cpp
	$(CXX) -o gt_alignment_test MIT/test/GT_AlignmentTest.cpp $(BASE_CXXFLAGS) -O2
	./gt_alignment_test

threading_test : MIT/test/TH_ThreadingTest.cpp
	$(CXX) -o threading_test MIT/test/TH_ThreadingTest.cpp \
	-DCONFIG_IMU -DCONFIG_STEREO $(CXXFLAGS) $(LDFLAGS) $(LDLIBS)
	./threading_test
	$(CXX) -o threading_test_custom MIT/test/TH_ThreadingTest.cpp \
	-DCONFIG_IMU -DCONFIG_STEREO -DBLAS_NUM_THREADS=2 -DOPENCV_NUM_THREADS=2 \
	-DACCELERATE_BLAS_SINGLE_THREADED=false $(CXXFLAGS) $(LDFLAGS) $(LDLIBS)
	./threading_test_custom

map_lock_test : MIT/test/LG_MapLockTimingTest.cpp
	$(CXX) -o map_lock_test MIT/test/LG_MapLockTimingTest.cpp $(BASE_CXXFLAGS) -O2 -pthread
	./map_lock_test

descriptor_cache_test : MIT/test/PT_DescriptorCacheTest.cpp
	$(CXX) -o descriptor_cache_test MIT/test/PT_DescriptorCacheTest.cpp \
	-DCONFIG_IMU -DCONFIG_STEREO $(BASE_CXXFLAGS) -O2 $(LDFLAGS) $(LDLIBS)
	./descriptor_cache_test
	$(CXX) -o descriptor_cache_sanitizer_test MIT/test/PT_DescriptorCacheTest.cpp \
	-DPT_DESCRIPTOR_CACHE_ONLY $(BASE_CXXFLAGS) -O1 -g -fsanitize=address,undefined
	./descriptor_cache_sanitizer_test

tracking_snapshot_test : MIT/test/MAP_TrackingSnapshotTest.cpp
	$(CXX) -o tracking_snapshot_test MIT/test/MAP_TrackingSnapshotTest.cpp \
	-DCONFIG_IMU -DCONFIG_STEREO $(BASE_CXXFLAGS) -O2 -ffunction-sections -fdata-sections \
	$(LDFLAGS) $(OP_TEST_LDFLAGS) $(LDLIBS)
	./tracking_snapshot_test

clean :
	rm -f main imu_test op_step3_test no_gt_init_test viz_export_test gt_alignment_test threading_test threading_test_custom map_lock_test descriptor_cache_test descriptor_cache_sanitizer_test tracking_snapshot_test

