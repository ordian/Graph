CC=g++
CFLAGS=-c -Wall -Wextra -std=c++03 -pedantic -Wunused -Wmissing-declarations -Wpointer-arith -Wcast-align -Wwrite-strings -Wredundant-decls
OFLAGS=-O3
LDFLAGS= 
TSFLAGS=-lboost_unit_test_framework

CPP_SRCS=src/graph.cpp src/algorithm.cpp src/main.cpp 
HPP_SRCS=include/graph.hpp include/algorithm.hpp include/priority_queue.hpp include/timer.hpp include/bitmap_image.hpp
TEST_SRCS=tests/testAlgorithm.cpp #$(wildcard tests/*.cpp)
OBJ_FILES=$(CPP_SRCS:.cpp=.o) 
TARGET=Graph


all: $(CPP_SRCS) $(TARGET) 

$(TARGET): $(OBJ_FILES)
	@echo "Linking..." $(OBJ_FILES)
	$(CC) $(LDFLAGS) $(OBJ_FILES) -o $@

.cpp.o: 
	@echo "Compiling..." $<
	$(CC) $(CFLAGS) $(OFLAGS) $< -o $@

test: all_tests
	$(CC) tests/testAlgorithm.o src/graph.o src/algorithm.o -o Test $(TSFLAGS)

all_tests: $(TEST_SRCS)
	$(CC) $(CFLAGS) $(OFLAGS) $(TEST_SRCS) -o tests/testAlgorithm.o

clean:
	@echo "Cleaning..."
	rm -rf src/*.o tests/*.o src/*~ include/*~ *~
 
mrproper: clean
	rm -rf ${TARGET} Test
 
.PHONY: clean
