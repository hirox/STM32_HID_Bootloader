CC=gcc
CXX=g++

VID_HI  = 0x12
VID_LOW = 0x09
PID_HI  = 0xBE
PID_LOW = 0xBA

FIRMWARE_KEY1 = 0x0123456789ABCDEF
FIRMWARE_KEY2 = 0xFEDCBA9876543210

-include configuration.mk

COMMON_FLAGS = -ggdb -c -Wall -DDEBUG_PRINTF
COMMON_FLAGS += -DFIRMWARE_KEY1=$(FIRMWARE_KEY1) -DFIRMWARE_KEY2=$(FIRMWARE_KEY2)
COMMON_FLAGS += -DVID_HI=$(VID_HI) -DVID_LOW=$(VID_LOW) -DPID_HI=$(PID_HI) -DPID_LOW=$(PID_LOW)

CFLAGS = $(COMMON_FLAGS)
CFLAGS += -std=gnu17

CXXFLAGS = $(COMMON_FLAGS)
CXXFLAGS += -std=c++20

LDFLAGS=-lstdc++
SOURCES=
CXX_SRCS=crypt.cc main.cc
INCLUDE_DIRS=-I .

vpath %.cc $(sort $(dir $(CXX_SRCS)))

ifeq ($(OS),Windows_NT)
	SOURCES+=hid-win.c
	LIBS=-lsetupapi -lhid
else
	UNAME_S := $(shell uname -s)
	ifeq ($(UNAME_S),Darwin)
		SOURCES+=hid-mac.c
		LIBS=-framework IOKit -framework CoreFoundation
	else
		SOURCES+=hid-libusb.c
		LIBS=`pkg-config libusb-1.0 --libs` -lrt -lpthread
		INCLUDE_DIRS+=`pkg-config libusb-1.0 --cflags`
		LDFLAGS+=-no-pie
		endif
endif

OBJECTS=$(SOURCES:.c=.c.o) $(CXX_SRCS:.cc=.cc.o)

EXECUTABLE = hid-flash

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CXX) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $@

%.c.o: %.c
	$(CC) $(CFLAGS) $(INCLUDE_DIRS) -c "$<" -o "$@"

%.cc.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDE_DIRS) -c "$<" -o "$@"

clean:
	rm -f $(OBJECTS) $(EXECUTABLE) $(EXECUTABLE).exe
