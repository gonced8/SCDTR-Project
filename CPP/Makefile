CC = g++
LDIR = boost_1_71_0
CFLAGS = -Wall -std=c++11 -O3 -I$(LDIR) -pthread

.PHONY: all run clean

all: ASIO
	
ASIO: ASIO2.cpp ASIO2.h
	$(CC) $(CFLAGS) -o ASIO2 ./ASIO2.cpp

run: all
	./ASIO2

clean:
	rm -rf ASIO2
