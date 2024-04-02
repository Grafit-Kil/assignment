CXX = g++
NVCC = nvcc
CXXFLAGS = -std=c++17 -O3 -Wall -Wextra
NVCCFLAGS = -arch=sm_50

all: main

main: main.o
	$(NVCC) $(NVCCFLAGS) $^ -o $@

main.o: main.cu
	$(NVCC) $(NVCCFLAGS) -c $< -o $@

clean:
	rm -rf *.o main
