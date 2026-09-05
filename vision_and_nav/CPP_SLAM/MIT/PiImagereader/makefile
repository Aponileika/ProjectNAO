CXX := clang++

TARGET := PantoReceiver
SRC := main.cpp

OPENCV := $(shell brew --prefix opencv)

OPENCVLIBS := \
	-lopencv_core \
	-lopencv_highgui \
	-lopencv_imgproc \
	-lopencv_imgcodecs

BASE_CXXFLAGS := \
	-Wall \
	-Wextra \
	-Wno-deprecated-declarations \
	-std=c++20 \
	-I$(OPENCV)/include/opencv4

CXXFLAGS := $(BASE_CXXFLAGS) -O2

LDFLAGS := \
	-L$(OPENCV)/lib

LDLIBS := $(OPENCVLIBS)

ifeq ($(DEBUG),1)
	CXXFLAGS := $(BASE_CXXFLAGS) \
		-O0 \
		-g \
		-fsanitize=address \
		-fno-omit-frame-pointer \
		-DDEBUG

	LDFLAGS += -fsanitize=address
endif

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS) $(LDLIBS)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)
