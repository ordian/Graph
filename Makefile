TARGET  := $(shell basename $$PWD)
# Compiler
CC      := g++
# Warning levels
WARN    := -Wall -Wextra -Werror -pedantic -Wunused -Wmissing-declarations -Wpointer-arith -Wcast-align -Wwrite-strings -Wredundant-decls
# Optimisation
OFLAGS  := -O2
# Aditionnal libraries to link
LDFLAGS := 
 
CPP_SRCS    = $(wildcard src/*.cpp)
OBJ_FILES   = $(CPP_SRCS:.cpp=.o) 
 
%o: src/%.cpp include/*.hpp
	@echo "Compiling "$<"..."
	$(CC) -c $(OFLAGS) $< -o $@
 
$(TARGET): $(OBJ_FILES)
	@echo "Linking..."
	$(CC)  $(OFLAGS) $(LDFLAGS) -o $@ $(OBJ_FILES)
	@echo "Done."
 
all: ${TARGET}
 
clean:
	@echo "Cleaning..."
	rm -rf *.o
 
mrproper: clean
	rm -rf ${TARGET}	
 
.PHONY: clean
