#include "ASIO2.h"


void open_port(){
	sp.open("/dev/cu.usbmodem14201", ec);
	while(ec){
		std::cout << "Could not open serial port" << std::endl;
		std::cout << "Please connect USB to the correct port" << std::endl;
		std::cout << "Will try again in 5 seconds" << std::endl;
		sleep(5);
		sp.open("/dev/cu.usbmodem14201", ec);
	}
	sp.set_option(serial_port_base::baud_rate(115200), ec);
}

void timer_handler(const error_code &ec)
{
	//timer expired – Launch new read
	async_read_until(sp, arduinocomms_buff, '\n', read_handler);
}

/*void write_handler(const error_code &ec, size_t nbytes) {
	//writer done – program new deadline
	tim.expires_from_now(boost::posix_time::seconds(5));
	tim.async_wait(timer_handler);
}*/

void read_handler(const error_code &ec, size_t nbytes) {
	//data is now available at read_buf
	float datalocal[paramNumber+1]; // All data, first being luminaire number
	measure measurelocal;
	std::stringstream mybuffer;
	char msg;
	mybuffer.str(std::string()); //Clear buffer
	mybuffer << &arduinocomms_buff;
	msg = mybuffer.str().at(0);
	//memcpy(&read_value, msg, sizeof(float));
	//std::cout << "Float " << read_value << std::endl;
	/*if(msg == '!'){ // Different message
		prettyPrint(mybuffer.str())
	}
	else{
		datalocal[0] = static_cast<float>(msg);
		memcpy(&datalocal[1], mybuffer.str().substr(1), sizeof(float));
		memcpy(&datalocal[2], mybuffer.str().substr(5), sizeof(float));
		for(int i=0; i<paramNumber;i++){
				data[static_cast<int>(datalocal[0])*paramNumber+(i)] = datalocal[i+1];
		}
	}
	measurelocal.current_time = time(NULL);
	measurelocal.measured_illuminance = datalocal[1];
	measurelocal.duty_cycle = datalocal[2];
	if(realtime[static_cast<int>(datalocal[0])*2+0]){
		std::cout << "s l " << static_cast<int>(datalocal[0]) << " " << datalocal[1] << " " << ctime(& measurelocal.current_time);
	}
	if(realtime[static_cast<int>(datalocal[0])*2+1]){
		std::cout << "s d " << static_cast<int>(datalocal[0]) << " " << datalocal[2] << " " << ctime(& measurelocal.current_time);
	}
	bufferqueue[static_cast<int>(datalocal[0])].push(measurelocal);
	if(bufferqueue[static_cast<int>(datalocal[0])].size() > 100) // More than one minute
		bufferqueue[static_cast<int>(datalocal[0])].pop();*/
	std::cout << mybuffer.str();
	tim.expires_from_now(boost::posix_time::milliseconds(100));
	tim.async_wait(timer_handler);
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
	tim.expires_from_now(boost::posix_time::milliseconds(100));
	tim.async_wait(timer_handler);
	//program chain of read operations
	//async_read_until(sp,arduinocomms_buff,'\n',read_handler);
	start_read_input();
	io.run(); //get things rolling
}
