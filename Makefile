CC=g++
CFLAGS=-c -Wall -Werror -pedantic -g # -Weffc++
LDFLAGS=#-I~/boost/boost_1_54_0
SOURCES=main.cpp graph.cpp algorithm.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=main

all: $(SOURCES) $(EXECUTABLE)
    
$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(OBJECTS) -o $@

.cpp.o:
	$(CC) $(LDFLAGS) $(CFLAGS) $< -o $@

clean:
	rm -rf *.o *~ main
