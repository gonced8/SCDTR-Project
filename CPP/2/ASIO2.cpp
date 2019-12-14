#include "ASIO2.h"


void open_port(){
	sp.open("/dev/ttyACM0", ec);
	while(ec){
		std::cout << "Could not open serial port" << std::endl;
		std::cout << "Please connect USB to the correct port" << std::endl;
		std::cout << "Will try again in 5 seconds" << std::endl;
		sleep(5);
		sp.open("/dev/ttyACMO", ec);
	}
	sp.set_option(serial_port_base::baud_rate(115200), ec);
}

void timer_handler(const error_code &ec)
{
	//timer expired – launch new write operation
	std::ostringstream os;
	os << "Counter = " << ++counter;
	async_write(sp, buffer(os.str()), write_handler);
}

void write_handler(const error_code &ec, size_t nbytes) {
	//writer done – program new deadline
	tim.expires_from_now(boost::posix_time::seconds(5));
	tim.async_wait(timer_handler);
}

void read_handler(const error_code &ec, size_t nbytes) {
	//data is now available at read_buf
	std::cout << &arduinocomms_buff;
	//program new read cycle
	async_read_until(sp,arduinocomms_buff,'\n',read_handler);
}

void start_read_input() {
	async_read_until(stm_desc, pccomms_buff, '\n', handle_read_input);
}

void handle_read_input (const error_code & ec, size_t len) {
	std::cout << "PC " << &pccomms_buff << std::endl;
	start_read_input();
}

int main() {
	open_port();
	//program timer for write operations
	tim.expires_from_now(boost::posix_time::seconds(5));
	tim.async_wait(timer_handler);
	//program chain of read operations
	async_read_until(sp,arduinocomms_buff,'\n',read_handler);
	io.run(); //get things rolling
}
