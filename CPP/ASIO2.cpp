#include "ASIO2.h"
#include "comm_codes.h"

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
	sp.open("/dev/cu.usbmodem14101", ec);
	while(ec){
		std::cout << "Could not open serial port" << std::endl;
		std::cout << "Please connect USB to the correct port" << std::endl;
		std::cout << "Will try again in 1 seconds" << std::endl;
		sleep(1);
		sp.open("/dev/cu.usbmodem14101", ec);
	}
	sp.set_option(serial_port_base::baud_rate(115200), ec);
}

void timer_handler(const error_code &ec){
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
	std::cout << mybuffer.str();
	msg = mybuffer.str().at(0);
	//std::cout << "Float " << read_value << std::endl;
	if(msg == '!'){ // Different message
		prettyPrint(mybuffer.str());
	}
	else if(msg == '*'){
		msg = mybuffer.str().at(1);
		datalocal[0] = static_cast<float>(msg);
		memcpy(&datalocal[1], mybuffer.str().c_str()+2, sizeof(float));
		memcpy(&datalocal[2], mybuffer.str().c_str()+6, sizeof(float));
		for(int i=0; i<paramNumber;i++){
				data[static_cast<int>(datalocal[0])*paramNumber+(i)] = datalocal[i+1];
		}
		measurelocal.current_time = time(NULL);
		measurelocal.measured_illuminance = datalocal[1];
		measurelocal.duty_cycle = datalocal[2];
		if(current_occupation[static_cast<int>(datalocal[0])] > 0.95){
			measurelocal.current_ref = current_ref_occupied[static_cast<int>(datalocal[0])];
		}
		else{
			measurelocal.current_ref = current_ref_unoccupied[static_cast<int>(datalocal[0])];
		}
		if(realtime[static_cast<int>(datalocal[0])*2+0]){
			std::cout << "s l " << static_cast<int>(datalocal[0]) << " " << datalocal[1] << " " << ctime(& measurelocal.current_time);
		}
		if(realtime[static_cast<int>(datalocal[0])*2+1]){
			std::cout << "s d " << static_cast<int>(datalocal[0]) << " " << datalocal[2] << " " << ctime(& measurelocal.current_time);
		}
		bufferqueue[static_cast<int>(datalocal[0])].push(measurelocal);
		if(bufferqueue[static_cast<int>(datalocal[0])].size() > perminute) // More than one minute
			bufferqueue[static_cast<int>(datalocal[0])].pop();
		energy[static_cast<int>(datalocal[0])] += (datalocal[1]/100)*(60/perminute);

	}
	tim.expires_from_now(boost::posix_time::milliseconds(100));
	tim.async_wait(timer_handler);
}

void prettyPrint (std::string s){
	int messagenum;
	int luminaire;
	float multifunfloat;
	messagenum = (int) s.at(1);
	luminaire = (int) s.at(2);
	memcpy(&multifunfloat, s.c_str()+3, sizeof(float));
	switch(messagenum){
		case occupancy_ans:
			std::cout<<"o " << luminaire << " " << multifunfloat << std::endl;
			break;
		case lower_bound_occupied_ans:
			std::cout<<"O " << luminaire << " " << multifunfloat << std::endl;
			break;
		case lower_bound_unoccupied_ans:
			std::cout<<"U " << luminaire << " " << multifunfloat << std::endl;
			break;
		case current_lower_bound_ans:
			std::cout<<"L " << luminaire << " " << multifunfloat << std::endl;
			break;
		case current_external_ans:
			std::cout<<"x " << luminaire << " " << multifunfloat << std::endl;
			break;
		case current_reference_ans:
			std::cout<<"r " << luminaire << " " << multifunfloat << std::endl;
			break;
		case current_cost_ans:
			std::cout<<"c " << luminaire << " " << multifunfloat << std::endl;
			break;
		case time_since_restart_ans:
			std::cout<<"t " << luminaire << " " << multifunfloat << std::endl;
			break;
		case set_occupied_ans:
			if(multifunfloat > 0.9){
				std::cout<< "ack" << std::endl;
			}
			else{
				std::cout << "err" << std::endl;
			}
		case set_occupied_value_ans:
			if(multifunfloat > 0.9){
				std::cout<< "ack" << std::endl;
			}
			else{
				std::cout << "err" << std::endl;
			}
		case set_unoccupied_value_ans:
			if(multifunfloat > 0.9){
				std::cout<< "ack" << std::endl;
			}
			else{
				std::cout << "err" << std::endl;
			}
		case set_cost_ans:
			if(multifunfloat > 0.9){
				std::cout<< "ack" << std::endl;
			}
			else{
				std::cout << "err" << std::endl;
			}
		case set_restart_ans:
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
			std::cout << "Entrou \n";
			send_arduino(luminaire, occupancy_ask);
		}
		else if(s.at(2) == 'O'){
			send_arduino(luminaire, lower_bound_occupied_ask);
		}
		else if(s.at(2) == 'U'){
			send_arduino(luminaire, lower_bound_unoccupied_ask);
		}
		else if(s.at(2) == 'L'){
			send_arduino(luminaire, current_lower_bound_ask);
		}
		else if(s.at(2) == 'x'){
			send_arduino(luminaire, current_external_ask);
		}
		else if(s.at(2) == 'r'){
			send_arduino(luminaire, current_reference_ask);
		}
		else if(s.at(2) == 'c'){
			send_arduino(luminaire, current_cost_ask);
		}
		else if(s.at(2) == 'p'){
			if(luminaire != 0){
				std::cout << "p " << luminaire << " " << data[luminaire*paramNumber + 1]/100 << std::endl;
			}
			else{
				multifunfloat = 0;
				for(int i = 0; i < luminaireNumber; i++){
					multifunfloat += data[i*paramNumber + 1];
				}
				std::cout << "p T " << multifunfloat/100  << std::endl;
			}
		}
		else if(s.at(2) == 't'){
			send_arduino(luminaire, 11);
		}
		else if(s.at(2) == 'e'){
			if(luminaire != 0){
				std::cout << "e " << luminaire << " " << energy[luminaire] << std::endl;
			}
			else{
				multifunfloat = 0;
				for(int i = 0; i < luminaireNumber; i++){
					multifunfloat += energy[i];
				}
				std::cout << "e T " << multifunfloat  << std::endl;
			}
		}
		else if(s.at(2) == 'v'){
			if(luminaire != 0){
				multifunfloat = 0;
				int i = 0;
				for(Node *node=bufferqueue[luminaire].front(); node!=NULL; node=node->next()){
					multifunfloat += std::max((float)0, node->value().measured_illuminance - node->value().current_ref);
					i++;
				}
				multifunfloat/= i;
				std::cout << "v " << luminaire << " " << multifunfloat << std::endl;
			}
			else{
				multifunfloat = 0;
				int i = 0;
				for (int j = 0; j < luminaireNumber; j++){
					for(Node *node=bufferqueue[luminaire].front(); node!=NULL; node=node->next()){
						multifunfloat += std::max((float)0, node->value().measured_illuminance - node->value().current_ref);
						i++;
					}
				}
				multifunfloat /= (i/3);
				std::cout << "v T " << multifunfloat << std::endl;
			}
		}
		else if(s.at(2) == 'f'){
			if(luminaire != 0){
				multifunfloat = 0;
				int i = 0;
				for(Node *node=bufferqueue[luminaire].front(); node!=NULL; node=node->next()){
					if(node->next() != NULL && node->next()->next() != NULL){
						if((node->value().measured_illuminance - node->next()->value().measured_illuminance) * (node->next()->value().measured_illuminance - node->next()->next()->value().measured_illuminance) < 0){
							multifunfloat += (abs(node->value().measured_illuminance - node->next()->value().measured_illuminance) + abs(node->next()->value().measured_illuminance - node->next()->next()->value().measured_illuminance))*perminute/(2*60);
							i++;
						}
					}
				}
				multifunfloat /= i;
				std::cout << "f " << luminaire << " " << multifunfloat << std::endl;
			}
			else{
				multifunfloat = 0;
				int i = 0;
				for(int j = 0; j < luminaireNumber; j++){
					for(Node *node=bufferqueue[luminaire].front(); node!=NULL; node=node->next()){
						if(node->next() != NULL && node->next()->next() != NULL){
							if((node->value().measured_illuminance - node->next()->value().measured_illuminance) * (node->next()->value().measured_illuminance - node->next()->next()->value().measured_illuminance) < 0){
								multifunfloat += (abs(node->value().measured_illuminance - node->next()->value().measured_illuminance) + abs(node->next()->value().measured_illuminance - node->next()->next()->value().measured_illuminance))*perminute/(2*60);
								i++;
							}
						}
					}
				}
				multifunfloat /= (i/3);
				std::cout << "f T " << multifunfloat << std::endl;
			}
		}
	}
	else if(s.at(0) == 'o'){
		luminaire = std::stoi(s.substr(2));
		multifunfloat = std::stoi(s.substr(4));
		if(multifunfloat > 0.95){
			current_occupation[luminaire] = 1;
		}
		else{
			current_occupation[luminaire] = 0;
		}
		send_arduino(luminaire, set_occupied_ask, multifunfloat);
	}
	else if(s.at(0) == 'O'){
		luminaire = std::stoi(s.substr(2));
		multifunfloat = std::stof(s.substr(4));
		current_ref_occupied[luminaire] = multifunfloat;
		send_arduino(luminaire, set_occupied_value_ask, multifunfloat);
	}
	else if(s.at(0) == 'U'){
		luminaire = std::stoi(s.substr(2));
		multifunfloat = std::stof(s.substr(4));
		current_ref_unoccupied[luminaire] = multifunfloat;
		send_arduino(luminaire, set_unoccupied_value_ask, multifunfloat);
	}
	else if(s.at(0) == 'c'){
		luminaire = std::stoi(s.substr(2));
		multifunfloat = std::stof(s.substr(4));
		send_arduino(luminaire, set_cost_ask, multifunfloat);
	}
	else if(s.at(0) == 'r'){
		send_arduino(luminaire, set_restart_ask);
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
	else
		os << luminaire << " " << messagenum << " " << parameter << std::endl;

	async_write(sp, buffer(os.str()),
			[](const error_code &ec, size_t len){;}
				);
}

int main() {
	open_port();
	//program timer for read arduino operations
	tim.expires_from_now(boost::posix_time::milliseconds(100));
	tim.async_wait(timer_handler);
	start_read_input();
	io.run(); //get things rolling
}
