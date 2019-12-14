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
boost::asio::deadline_timer tim {io};
boost::asio::streambuf read_buf; //read buffer
boost::system::error_code ec;
int counter = 0;

//Functions
void write_handler(const error_code &ec, size_t nbytes);
void timer_handler(const error_code &ec);
void read_handler(const error_code &ec, size_t nbytes);
void open_port();

#endif
