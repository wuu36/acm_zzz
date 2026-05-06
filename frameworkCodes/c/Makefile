
# Source files
SRC = main.c utility.c

# Compiler settings
CC = gcc
CFLAGS = -Wall -O2 -I.

# Target
TARGET = main.exe

# Default target
all: $(TARGET)
	
# Build rule
$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $(SRC) -lm

# Run simulation
run: $(TARGET)
	./$(TARGET)


# Test
test:
	gcc -o test_rk4.exe test_rk4.c -lm
	./test_rk4.exe

	
# Clean
clean:
	rm -f *.0 $(TARGET)

# Phony targets
.PHONY: all run clean

