# Default to gcc for local testing; accepts Buildroot's TARGET_CC
CC ?= gcc

CFLAGS += -Wall -Wextra -pthread -O2
LIBS   += -pthread -lm

TARGET = fluke
SRC    = fluke.c
OBJ    = $(SRC:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)

.PHONY: all clean
