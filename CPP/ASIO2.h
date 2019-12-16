// PC console functions

#ifndef ASIO_H
#define ASIO_H

#include <iostream>
#include <string>
#include <sstream>
#include <boost/asio.hpp>
using namespace boost::system;
using namespace boost::asio;

#define paramNumber 2
#define luminaireNumber 3
#define perminute 100

struct measure{
  std::time_t current_time;
	float measured_illuminance;
	float duty_cycle;
  float current_ref;
};
typedef measure Value;

class Node {
	Value _value;
	Node *_next;
public:
	Node(const Value v);
	Value value();
	Node *next();
	void link(Node *n);
};

class LinkedList {
	Node *_front;
	Node *_back;
	unsigned int _size;
public:
  LinkedList();
  Node *front();
  Node *back();
  bool empty();
  unsigned int size();
  void push (const Value value);
  void pop();
};

//GLOBALS
boost::asio::io_context io;
boost::asio::serial_port sp {io};
boost::asio::posix::stream_descriptor stm_desc {io, ::dup(STDIN_FILENO)};
boost::asio::deadline_timer tim {io};
boost::asio::streambuf arduinocomms_buff {256}; //arduino buffer
boost::asio::streambuf pccomms_buff {1024};
boost::system::error_code ec;
bool realtime[luminaireNumber*2] = {false, false, false, false, false, false};
LinkedList bufferqueue[luminaireNumber];
float data[luminaireNumber*paramNumber];
float energy[luminaireNumber] = {0, 0, 0};
float current_ref_occupied[luminaireNumber] = {0, 0, 0};
float current_ref_unoccupied[luminaireNumber] = {0, 0, 0};
float current_occupation[luminaireNumber] = {0, 0, 0};
float read_value;

//void write_handler(const error_code &ec, size_t nbytes);
void timer_handler(const error_code &ec);
void read_handler(const error_code &ec, size_t nbytes);
void prettyPrint (char *msg);
void start_read_input(const error_code & ec, size_t len);
void handle_read_input (const error_code & ec, size_t len);
void choose_case(std::string s);
void send_arduino(int luminaire, int messagenum, float parameter = -1);
void open_port();

#endif
