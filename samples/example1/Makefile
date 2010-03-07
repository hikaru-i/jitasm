CC	= g++
CFLAGS	= -mmmx -msse -msse2 -march=i686 -fno-operator-names -Wall -I../../
TARGET	= example1

all: $(TARGET)

example1:
	$(CC) $(CFLAGS) example1.cpp -o $@

clean:
	rm -rf *.o $(TARGET)
