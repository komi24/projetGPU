# extra flags pour le link
LDFLAGS =

# Flags pour la compilation
CFLAGS =

CX = g++
NVCC=/usr/local/cuda-6.5/bin/nvcc  -ccbin g++ -m64
NVCCFLAGS= -arch sm_12

## liste des exécutables à créer
SRC = $(wildcard *.cu)
OBJ = $(SRC:.c=.o)
EXEC = cuda_info matrix-simple

all : $(EXEC)

cuda_info: cuda_info.o
	$(NVCC) $(LDFLAGS)  -o $@ $+

matrix-simple: matrix-simple.o
	$(NVCC) $(LDFLAGS)  -o $@ $+

%.o: %.cpp
	$(CX) -c $< $(CFLAGGS) -o $@

%.o: %.cu
	$(NVCC) $(NVCCFLAGS) -c $<

clean:
		rm -f *.o

mrproper: clean
		rm $(EXEC)


