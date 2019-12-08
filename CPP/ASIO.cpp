//ASYNC_CONSOLE
#include <iostream>
#include <boost/asio.hpp>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <sstream>

using ec = const boost::system::error_code;
using sz = std::size_t;
using namespace boost::system;
using namespace boost::asio;
using boost::asio::chrono::seconds;
//BEGIN CLASS ASYNC_CONSOLE
class async_console {
	boost::asio::steady_timer tim;
	boost::asio::serial_port sp;
	boost::asio::posix::stream_descriptor stm_desc;
	boost::asio::streambuf pccomms_buff;
	boost::asio::streambuf arduinocomms_buff;
	float read_value;
public:
		void start() {
			//open_port();
			start_timer();
			//start_read_arduino();
			start_read_input();
		}
		async_console (boost::asio::io_context &io)
		: tim {io},
		  sp {io},
		  stm_desc {io, ::dup(STDIN_FILENO)},
		  pccomms_buff {1024},
		  arduinocomms_buff{4}
		{
			start();
		}
		/*void open_port(){
			sp.open("/dev/ttyACM0", ec);
			while(ec){
				std::cout << "Could not open serial port" << std::endl;
				std::cout << "Please connect USB to the correct port" << std::endl;
				std::cout << "Will try again in 5 seconds" << std::endl;
				sleep(5);
				sp.open("/dev/ttyACMO", ec)
			}
			sp.set_option(serial_port_base::baud_rate(9600),ec);
		}*/
		void start_timer() {
			tim.expires_from_now( std::chrono::seconds(5) );
			tim.async_wait([this] (ec & err) {
				start_read_arduino();
				start_timer();
			});
		}
		void start_read_arduino() {
			async_read_until(stm_desc, arduinocomms_buff, '\n', [this](ec& err, sz len){
				process_arduinoinput();
			});
		}
		void start_read_input() {
			async_read_until(stm_desc, pccomms_buff, '\n', [this](ec& err, sz len){
				process_pcinput();
			});
		}
		void process_pcinput() {
			// Process first
			std::cout << "Echo Function" << std::endl;
			std::cout << &pccomms_buff << std::endl;
			// Call the read again
			start_read_input();
		}
		void process_arduinoinput() {
			// Convert read value to float
			std::stringstream mybuffer;
			char msg[4];
			mybuffer << &arduinocomms_buff;
			strncpy (msg, mybuffer.str().c_str(), sizeof(msg));
			memcpy(&read_value, msg, sizeof(float));
			std::cout << "Float " << read_value << std::endl;
			mybuffer.str(std::string()); //Clear buffer
		}
}; //END CLASS ASYNC_CONSOLE

int main (int argc, char* argv[]) {
	boost::asio::io_context io;
	async_console ac{io};
	io.run();
}
