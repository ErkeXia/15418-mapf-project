CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra

SRCS := main.cpp grid.cpp planner_serial.cpp cbs.cpp
OBJS := $(SRCS:.cpp=.o)

mapf: $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o mapf

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) mapf
