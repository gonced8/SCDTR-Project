CC = g++
LDIR1 = boost_1_71_0
LDIR2 = ../Arduino/
CFLAGS = -Wall -std=c++11 -O3 -I$(LDIR1) -I$(LDIR2) -pthread

.PHONY: all run clean

all: ASIO
	
ASIO: ASIO.cpp ASIO.h
	$(CC) $(CFLAGS) -o ASIO ./ASIO.cpp

run: all
	./ASIO

clean:
	rm -rf ASIO
