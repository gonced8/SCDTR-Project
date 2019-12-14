#include "ASIO2.h"

Node::Node(const Value v){
	_value = v;
	_next = NULL;
}
Value Node::value(){
	return _value;
}
Node* Node::next(){
	return _next;
}
void Node::link(Node* n){
	_next = n;
}

LinkedList::LinkedList(){
	_front = NULL;
	_back = NULL;
	_size = 0;
}
Node* LinkedList::front(){
	return _front;
}
Node* LinkedList::back(){
	return _back;
}
bool LinkedList::empty(){
	return _size==0;
}
unsigned int LinkedList::size(){
	return _size;
}
void LinkedList::push(const Value value){
	Node *temp = new Node(value);
	if(_back != NULL){
		_back->link(temp);
		_back = _back->next();
	}
	else
		_front = _back = temp;
	_size++;
}
void LinkedList::pop(){
	Node *temp = _front;
	_front = temp->next();
	delete temp;
	_size--;
}
void printList(LinkedList toprint, int argument) {
  measure a;
  for(Node *node=toprint.front(); node!=NULL; node=node->next()){
    a = node->value();
    if (argument == 1)
      std::cout << ctime(& a.current_time) << " " << a.measured_illuminance << std::endl;
    if (argument == 2)
      std::cout << ctime(& a.current_time) << " " << a.duty_cycle << std::endl;
  }
}

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
		prettyPrint(mybuffer.str());
	}
	else{
		datalocal[0] = static_cast<float>(msg);
		//memcpy(&datalocal[1], mybuffer.str().substr(1), sizeof(float));
		//memcpy(&datalocal[2], mybuffer.str().substr(5), sizeof(float));
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

void prettyPrint (std::string s){
	int messagenum;
	int luminaire;
	float multifunfloat;
	messagenum = (int) s.at(1);
	luminaire = (int) s.at(2);
	multifunfloat = 10.0;
	//memcpy(&multifunfloat, s.substr(3), sizeof(float));
	switch(messagenum){
		case 3:
			std::cout<<"o " << luminaire << " " << multifunfloat << std::endl;
			break;
		case 4:
			std::cout<<"O " << luminaire << " " << multifunfloat << std::endl;
			break;
		case 5:
			std::cout<<"U " << luminaire << " " << multifunfloat << std::endl;
			break;
		case 6:
			std::cout<<"L " << luminaire << " " << multifunfloat << std::endl;
			break;
		case 7:
			std::cout<<"x " << luminaire << " " << multifunfloat << std::endl;
			break;
		case 8:
			std::cout<<"r " << luminaire << " " << multifunfloat << std::endl;
			break;
		case 9:
			std::cout<<"c " << luminaire << " " << multifunfloat << std::endl;
			break;
		case 10:
			if(luminaire != 0){
				std::cout<<"p " << luminaire << " " << multifunfloat << std::endl;
			}
			else{
				std::cout<<"p T " << multifunfloat << std::endl;
			}
			break;
		case 11:
			std::cout<<"t " << luminaire << " " << multifunfloat << std::endl;
			break;
		case 12:
			if(luminaire != 0){
				std::cout<<"e " << luminaire << " " << multifunfloat << std::endl;
			}
			else{
				std::cout<<"e T " << multifunfloat << std::endl;
			}
			break;
		case 13:
			if(luminaire != 0){
				std::cout<<"v " << luminaire << " " << multifunfloat << std::endl;
			}
			else{
				std::cout<<"v T " << multifunfloat << std::endl;
			}
			break;
		case 14:
			if(luminaire != 0){
				std::cout<<"f " << luminaire << " " << multifunfloat << std::endl;
			}
			else{
				std::cout<<"f T " << multifunfloat << std::endl;
			}
			break;
		case 15:
			if(luminaire != 0){
				std::cout<<"p " << luminaire << " " << multifunfloat << std::endl;
			}
			else{
				std::cout<<"p T " << multifunfloat << std::endl;
			}
			break;
		case 16:
			if(multifunfloat > 0.9){
				std::cout<< "ack" << std::endl;
			}
			else{
				std::cout << "err" << std::endl;
			}
		case 17:
			if(multifunfloat > 0.9){
				std::cout<< "ack" << std::endl;
			}
			else{
				std::cout << "err" << std::endl;
			}
		case 18:
			if(multifunfloat > 0.9){
				std::cout<< "ack" << std::endl;
			}
			else{
				std::cout << "err" << std::endl;
			}
		case 19:
			if(multifunfloat > 0.9){
				std::cout<< "ack" << std::endl;
			}
			else{
				std::cout << "err" << std::endl;
			}
	}
}


void start_read_input() {
	async_read_until(stm_desc, pccomms_buff, '\n', handle_read_input);
}

void handle_read_input (const error_code & ec, size_t len) {
	std::stringstream mybuffer;
	mybuffer.str(std::string()); //Clear buffer
	mybuffer << &pccomms_buff;
	std::string s = mybuffer.str();
	std::cout << "Inside PC\n";
	choose_case(s);
	start_read_input();
}

void choose_case(std::string s) {
	int luminaire;
	float multifunfloat;
	int messagenum;
	if(s.at(0) == 'g'){
		try {
			luminaire = std::stoi(s.substr(4));
		}
		catch(std::invalid_argument& e){
			luminaire = 0; // All luminaires
		}
		if(s.at(2) == 'l'){
			std::cout << "l " << luminaire << " " << data[luminaire*paramNumber + 0] << std::endl;
		}
		else if(s.at(2) == 'd'){
			std::cout << "d " << luminaire << " " << data[luminaire*paramNumber + 1] << std::endl;
		}
		else if(s.at(2) == 'o'){
			send_arduino(luminaire, 3);
		}
		else if(s.at(2) == 'O'){
			send_arduino(luminaire, 4);
		}
		else if(s.at(2) == 'U'){
			send_arduino(luminaire, 5);
		}
		else if(s.at(2) == 'L'){
			send_arduino(luminaire, 6);
		}
		else if(s.at(2) == 'x'){
			send_arduino(luminaire, 7);
		}
		else if(s.at(2) == 'r'){
			send_arduino(luminaire, 8);
		}
		else if(s.at(2) == 'c'){
			send_arduino(luminaire, 9);
		}
		else if(s.at(2) == 'p'){
			send_arduino(luminaire, 10);
		}
		else if(s.at(2) == 't'){
			send_arduino(luminaire, 11);
		}
		else if(s.at(2) == 'e'){
			send_arduino(luminaire, 12);
		}
		else if(s.at(2) == 'v'){
			send_arduino(luminaire, 13);
		}
		else if(s.at(2) == 'f'){
			send_arduino(luminaire, 14);
		}
	}
	else if(s.at(0) == 'o'){
		luminaire = std::stoi(s.substr(2));
		multifunfloat = std::stoi(s.substr(4));
		send_arduino(luminaire, 15, multifunfloat);
	}
	else if(s.at(0) == 'O'){
		luminaire = std::stoi(s.substr(2));
		multifunfloat = std::stof(s.substr(4));
		send_arduino(luminaire, 16, multifunfloat);
	}
	else if(s.at(0) == 'U'){
		luminaire = std::stoi(s.substr(2));
		multifunfloat = std::stof(s.substr(4));
		send_arduino(luminaire, 17, multifunfloat);
	}
	else if(s.at(0) == 'c'){
		luminaire = std::stoi(s.substr(2));
		multifunfloat = std::stof(s.substr(4));
		send_arduino(luminaire, 18, multifunfloat);
	}
	else if(s.at(0) == 'r'){
		send_arduino(luminaire, 19);
	}
	else if(s.at(0) == 'b'){
		if(s.at(2) == 'l'){
			luminaire = std::stoi(s.substr(4));
			printList(bufferqueue[luminaire], 1);
		}
		else if(s.at(2) == 'd'){
			luminaire = std::stoi(s.substr(4));
			printList(bufferqueue[luminaire], 2);
		}
	}
	else if(s.at(0) == 's'){
		if(s.at(2) == 'l'){
			luminaire = std::stoi(s.substr(4));
			realtime[luminaire*2+0] = !realtime[luminaire*2+0];
		}
		else if(s.at(2) == 'd'){
			luminaire = std::stoi(s.substr(4));
			realtime[luminaire*2+1] = !realtime[luminaire*2+1];
		}
	}
}

void send_arduino(int luminaire, int messagenum, float parameter){
	std::ostringstream os;
	if(parameter == -1)
		 os << luminaire << " " << messagenum << std::endl;
	else{
		os << luminaire << " " << messagenum << " " << parameter << std::endl;
		async_write(sp, buffer(os.str()),
			[](const error_code &ec, size_t len){
					;
			}
		);
	}
}

int main() {
	open_port();
	//program timer for read arduino operations
	tim.expires_from_now(boost::posix_time::milliseconds(100));
	tim.async_wait(timer_handler);
	start_read_input();
	io.run(); //get things rolling
}
