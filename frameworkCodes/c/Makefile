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
TARGET_FOC_LOAD = foc_load_test.exe
TARGET_VELOCITY = velocity_test.exe

# Default target
all: $(TARGET)
	
# Build rule
$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $(SRC) -lm

# FOC test simulation
$(TARGET_FOC): main_foc_test.c $(UTIL_SRC) $(CTRL_SRC)
	$(CC) $(CFLAGS) -o $@ main_foc_test.c $(UTIL_SRC) $(CTRL_SRC) -lm
	
# FOC with load test simulation
$(TARGET_FOC_LOAD): main_foc_load_test.c $(UTIL_SRC) $(CTRL_SRC)
	$(CC) $(CFLAGS) -o $@ main_foc_load_test.c $(UTIL_SRC) $(CTRL_SRC) -lm

# Velocity test simulation
$(TARGET_VELOCITY): main_velocity_test.c $(UTIL_SRC) $(CTRL_SRC)
	$(CC) $(CFLAGS) -o $@ main_velocity_test.c $(UTIL_SRC) $(CTRL_SRC) -lm

# Run simulation
run: $(TARGET)
	./$(TARGET)

# Build and run FOC test
foc_test: $(TARGET_FOC)
	./$(TARGET_FOC)

# Build and run FOC with load test
foc_load_test: $(TARGET_FOC_LOAD)
	./$(TARGET_FOC_LOAD)

# Build and run velocity test
velocity_test: $(TARGET_VELOCITY)
	./$(TARGET_VELOCITY)


# Test
test:
	gcc -o test_rk4.exe test_rk4.c -lm
	./test_rk4.exe
	
# Clean
clean:
	rm -f *.o $(TARGET) $(TARGET_FOC)

# Phony targets
.PHONY: all run clean

