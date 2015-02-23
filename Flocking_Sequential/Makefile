CXX = g++
CXXFLAGS = -O2 -fopenmp -Wall
SRCS = $(wildcard *.cxx)
PROG = boids
OBJS = $(SRCS:%.cxx=%.o)
LDFLAGS = -lm -g -pg -fopenmp -o $(PROG)

$(PROG): $(OBJS)
	$(CXX) $(OBJS) $(LDFLAGS)

%.o:%.cxx
	$(CXX) -c $(CXXFLAGS) $<

test: $(PROG)
	./$(PROG)

clean:
	rm -f $(OBJS) $(PROG) *~
