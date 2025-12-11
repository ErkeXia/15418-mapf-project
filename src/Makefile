CXX     := g++
NVCC    := nvcc
CXXFLAGS := -std=c++17 -O2
NVCCFLAGS := -std=c++17 -O2

SRC_CPP := main.cpp grid.cpp planner_serial.cpp cbs.cpp
SRC_CU  := planner_bfs.cu

OBJ_CPP := $(SRC_CPP:.cpp=.o)
OBJ_CU  := $(SRC_CU:.cu=.o)

TARGET  := cbs

all: $(TARGET)

# Compile C++ files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile CUDA file
%.o: %.cu
	$(NVCC) $(NVCCFLAGS) -c $< -o $@

# Link everything with nvcc (so CUDA runtime is linked automatically)
$(TARGET): $(OBJ_CPP) $(OBJ_CU)
	$(NVCC) $(NVCCFLAGS) $^ -o $@

clean:
	rm -f $(OBJ_CPP) $(OBJ_CU) $(TARGET)
