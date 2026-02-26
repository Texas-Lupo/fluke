# Default to gcc for local testing, but accept Buildroot's TARGET_CC
CC ?= gcc

# Append necessary pthread library to compiler and linker flags
CFLAGS += -Wall -Wextra -pthread
LIBS += -pthread

TARGET = fluke
SRC = fluke.c
OBJ = $(SRC:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)

.PHONY: all clean
