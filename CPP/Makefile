CC = g++
LDIR1 = boost_1_71_0
LDIR2 = ../session3/
CFLAGS = -Wall -std=c++11 -O3 -I$(LDIR1) -I$(LDIR2) -pthread

.PHONY: all run clean

all: ASIO
	
ASIO: ASIO2.cpp ASIO2.h
	$(CC) $(CFLAGS) -o ASIO2 ./ASIO2.cpp

run: all
	./ASIO2

clean:
	rm -rf ASIO2
