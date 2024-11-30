CXX = clang++
CXXFLAGS = -std=c++20 -I/opt/homebrew/opt/opencv/include/opencv4
LDFLAGS = -L/opt/homebrew/opt/opencv/lib -lopencv_core -lopencv_highgui -lopencv_imgproc

TARGET = main
SRC = main.cpp

all: $(TARGET)  # Initially 'all: $(TARGET)' so that only make run actually compiles
            # and runs.  As 'all: run', simply typing 'make' will compile and run 'apple'
            # but, doing 'all: $(TARGET)' when 'make', will just compile, and not also ./apple

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS) -Wno-deprecated-anon-enum-enum-conversion

run: $(TARGET)
	./$(TARGET)

test: test.cpp
	$(CXX) $(CXXFLAGS) test.cpp -o test $(LDFLAGS) -Wno-deprecated-anon-enum-enum-conversion
	./test
	rm -f test

tester: tester.cpp
	$(CXX) $(CXXFLAGS) tester.cpp -o tester $(LDFLAGS) -Wno-deprecated-anon-enum-enum-conversion
	./tester
	rm -f tester

clean:
	rm -f $(TARGET)
	rm -f test