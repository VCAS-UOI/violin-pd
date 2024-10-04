
# Compiler and flags
CC = gcc
# on my old centos I need the -std=c99 flag, but then ftruncate is not found...
CFLAGS = -I./libCorrect

RCV_CFLAGS = -pthread
RCV_CFLAGS += -lserialport

# Directories
LIB_DIR = libCorrect
XMIT_DIR = xmit
RECV_DIR = recv
BUILD_DIR = build

# Library files 
LIB_SRC = $(wildcard $(LIB_DIR)/*.c)
LIB_OBJ = $(patsubst $(LIB_DIR)/%.c,$(BUILD_DIR)/%.o,$(LIB_SRC))

LIB_DEC_OBJ = $(BUILD_DIR)/decode.o $(BUILD_DIR)/reed-solomon.o $(BUILD_DIR)/polynomial.o
LIB_ENC_OBJ = $(BUILD_DIR)/encode.o $(BUILD_DIR)/reed-solomon.o $(BUILD_DIR)/polynomial.o

# Application files
XMIT_SRC = $(wildcard $(XMIT_DIR)/*.c)
XMIT_OBJ = $(patsubst $(XMIT_DIR)/%.c,$(BUILD_DIR)/%.o,$(XMIT_SRC))
XMIT_TARGET = $(BUILD_DIR)/xmit

RECV_SRC = $(wildcard $(RECV_DIR)/*.c)
RECV_OBJ = $(patsubst $(RECV_DIR)/%.c,$(BUILD_DIR)/%.o,$(RECV_SRC))
RECV_TARGET = $(BUILD_DIR)/recv

# Default target
all: $(RECV_TARGET) $(XMIT_TARGET)

xmit: $(XMIT_TARGET)

recv: $(RECV_TARGET)

# Create the build directory
$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

# Rule to build the library object file
$(BUILD_DIR)/%.o: $(LIB_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Rule to build the application object files
$(BUILD_DIR)/%.o: $(XMIT_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: $(RECV_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Rule to link the application
$(RECV_TARGET): $(RECV_OBJ) $(LIB_DEC_OBJ) 
	$(CC) $(CFLAGS) -o $@ $^ $(RCV_CFLAGS) 

$(XMIT_TARGET): $(XMIT_OBJ) $(LIB_ENC_OBJ) 
	$(CC) $(CFLAGS) -o $@ $^

# Clean up
clean:
	rm -rf $(BUILD_DIR)

# Phony targets
.PHONY: all clean

