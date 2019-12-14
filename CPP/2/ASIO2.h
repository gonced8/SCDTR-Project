// PC console functions

#ifndef ASIO_H
#define ASIO_H

#include <iostream>
#include <string>
#include <sstream>
#include <boost/asio.hpp>
using namespace boost::system;
using namespace boost::asio;
//GLOBALS
boost::asio::io_context io;
boost::asio::serial_port sp {io};
boost::asio::posix::stream_descriptor stm_desc {io, ::dup(STDIN_FILENO)};
boost::asio::deadline_timer tim {io};
boost::asio::streambuf arduinocomms_buff {256}; //arduino buffer
boost::asio::streambuf pccomms_buff {1024};
boost::system::error_code ec;
int counter = 0;

//Functions
void write_handler(const error_code &ec, size_t nbytes);
void timer_handler(const error_code &ec);
void read_handler(const error_code &ec, size_t nbytes);
void start_read_input(const error_code & ec, size_t len);
void handle_read_input (const error_code & ec, size_t len);
void open_port();

#endif
