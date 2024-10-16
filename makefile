# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -I/usr/include/eigen3

# Source files
SRC = main.cpp

# Output executable
OUT = UAV

# Build target
all: $(OUT)

# Link the object files to create the executable
$(OUT): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(OUT) $(SRC)

# Clean target
clean:
	rm -f $(OUT)