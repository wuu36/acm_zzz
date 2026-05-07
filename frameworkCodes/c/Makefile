# Usage:
#   make            - Build the default simulation
#   make load_test  - Build and run load test
#   make foc_test   - Build and run FOC test
#   make run        - Build and run default simulation
#   make clean      - Remove object files and executables



# Source files
SRC = main.c utility.c

# Common source
UTIL_SRC = utility.c
CTRL_SRC = main_switch.c

# Compiler settings
CC = gcc
CFLAGS = -Wall -O2 -I.

# Target
TARGET = main.exe

TARGET_FOC = foc_test.exe

# Default target
all: $(TARGET)
	
# Build rule
$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $(SRC) -lm

# FOC test simulation
$(TARGET_FOC): main_foc_test.c $(UTIL_SRC) $(CTRL_SRC)
	$(CC) $(CFLAGS) -o $@ main_foc_test.c $(UTIL_SRC) $(CTRL_SRC) -lm

# Run simulation
run: $(TARGET)
	./$(TARGET)

# Build and run FOC test
foc_test: $(TARGET_FOC)
	./$(TARGET_FOC)


# Test
test:
	gcc -o test_rk4.exe test_rk4.c -lm
	./test_rk4.exe

	
# Clean
clean:
	rm -f *.o $(TARGET) $(TARGET_FOC)

# Phony targets
.PHONY: all run clean

