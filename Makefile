CXX = clang++
CXXFLAGS = -std=c++20
LDFLAGS = -framework OpenGL -framework GLUT

TARGET = main
SRC = main.cpp

all: $(TARGET)  # Initially 'all: $(TARGET)' so that only make run actually compiles
            # and runs.  As 'all: run', simply typing 'make' will compile and run 'apple'
            # but, doing 'all: $(TARGET)' when 'make', will just compile, and not also ./apple

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS)

run: $(TARGET)
	./$(TARGET)

test: test.cpp
	$(CXX) $(CXXFLAGS) test.cpp -o test $(LDFLAGS)
	./test
	rm test

clean:
	rm -f $(TARGET)