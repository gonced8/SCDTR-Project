CC = g++
LDIR = boost_1_71_0
CFLAGS = -Wall -O3 -I$(LDIR) -pthread

.PHONY: all run clean

all: ASIO
	
ASIO: ASIO.cpp
	$(CC) $(CFLAGS) -o ASIO ./ASIO.cpp

run: all
	./ASIO

clean:
	rm -rf ASIO
