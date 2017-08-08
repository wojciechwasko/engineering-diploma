# Makefile programu uruchamianego na robocie.
# Copyright Wojciech Waśko 2011
# wojciechwasko@gmail.com

CXX = g++
LD = g++

CXX_OPTS = -g -I/usr/local/Aria/include
LD_OPTS = -g

LIBS = -lboost_system-mt -lboost_thread-mt -lboost_system-mt -lpthread -lAria -ldl -L/usr/local/Aria/lib

SRCS = src/server.cpp \
	src/driver.cpp \
	src/main.cpp \
	src/utils.cpp \
	src/message.cpp \
	src/steering_model.cpp

OBJS = $(patsubst src/%.cpp, build/%.o, $(SRCS))

LD_OPTS = $(LIBS)

all : server

server : $(OBJS)
	$(LD) $(LD_OPTS) $^ $(LD_LIBS) -o $@

build/%.o : src/%.cpp src/%.hpp
	$(CXX) $(CXX_OPTS) -c $< -o $@

build/%.o: src/%.cpp
	$(CXX) $(CXX_OPTS) -c $< -o $@

clean :
	rm -f $(OBJS) server

.PHONY : clean
