# linux-pca9865-lib Makefile
# This makefile build the linux-pca9865-lib library
# as both a static and shared library.
# The library is built from the source files in the src directory.

# Define the compiler toolchain (use CROSS_COMPILE variable to specify the toolchain)
CC = $(CROSS_COMPILE)gcc
CFLAGS = -Wall -Wextra -Werror -std=gnu99 -g

# Define the library name
LIB_NAME = libpca9865

# Define the library source files folder
LIB_SRC_DIR = src

# Define the library header files folder
LIB_INC_DIR = include lib/userspace-i2c-linux/include

# Define the library object files folder
LIB_OBJ_DIR = obj

# Define the static library output folder
LIB_STATIC_DIR = build/static

# Define the shared library output folder
LIB_SHARED_DIR = build/shared

# Define the library source files
LIB_SRC = $(wildcard $(LIB_SRC_DIR)/*.c)

# Define the library object files
LIB_OBJ = $(patsubst $(LIB_SRC_DIR)/%.c, $(LIB_OBJ_DIR)/%.o, $(LIB_SRC))

#define compilation targets
all: static shared

# Compile the library as a static library
static:
	@mkdir -p $(LIB_OBJ_DIR)
	@mkdir -p $(LIB_STATIC_DIR)
	$(CC) $(CFLAGS) -c $(LIB_SRC) $(foreach d,$(LIB_INC_DIR),-I$d) -o $(LIB_OBJ)
	ar rcs $(LIB_STATIC_DIR)/$(LIB_NAME).a $(LIB_OBJ)

# Compile the library as a shared library
shared:
	@mkdir -p $(LIB_OBJ_DIR)
	@mkdir -p $(LIB_SHARED_DIR)
	$(CC) $(CFLAGS) -fPIC -c $(LIB_SRC) $(foreach d,$(LIB_INC_DIR),-I$d) -o $(LIB_OBJ)
	$(CC) -shared -o $(LIB_SHARED_DIR)/$(LIB_NAME).so $(LIB_OBJ)

clean:
	rm -rf $(LIB_OBJ_DIR) $(LIB_STATIC_DIR) $(LIB_SHARED_DIR)

