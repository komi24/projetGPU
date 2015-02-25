NVCC=/usr/local/cuda-6.0/bin/nvcc -arch=sm_20 -ccbin g++ -m64
CXX = g++
CXXFLAGS =  -Wall
SRCS = $(wildcard *.cxx)
SRCS2 = $(wildcard *.cu)
PROG = boids
OBJS = $(SRCS:%.cxx=%.o)
OBJS2 = $(SRCS2:%.cu=%.o)

LDFLAGS = -lm -lcuda -g -pg -o $(PROG)

$(PROG): $(OBJS) $(OBJS2)
	$(NVCC) $(OBJS) $(OBJS2) $(LDFLAGS)

%.o:%.cxx
	$(NVCC) -c $<

%.o:%.cu
	$(NVCC) -c $<

test: $(PROG)
	./$(PROG)

clean:
	rm -f $(OBJS) $(OBJS2) $(PROG) *~
