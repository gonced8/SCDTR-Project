#include "ASIO.h"

struct timeval start;

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
	// Tries to open the port indefinitely
	char port_name[] = "/dev/cu.usbmodem143101";
	sp.open(port_name, ec);
	while(ec){
		std::cout << "Could not open serial port" << std::endl;
		std::cout << "Please connect USB to the correct port" << std::endl;
		std::cout << "Will try again in 1 seconds" << std::endl;
		sleep(1);
		sp.open(port_name, ec);
	}
	sp.set_option(serial_port_base::baud_rate(115200), ec);
}

void timer_handler(const error_code &ec){
	// Timer expired – Launch new read
	async_read_until(sp, arduinocomms_buff, '\n', read_handler);
}

void start_read_arduino(){
	async_read_until(sp, arduinocomms_buff, '\n', read_handler);
}

void read_handler(const error_code &ec, size_t nbytes) {
	// Arduino data is now available at read_buf
	// Lets first put it into a string to process it better
	char msg;
	std::istream is(&arduinocomms_buff);
	std::string line;
	std::getline(is, line);
	msg = line.at(0);

	float elapsed_time;

	if(msg == '?'){ // Different message, a special user request
		prettyPrint(line);
	}

	else if(msg == '!'){ // Message with logging purposes
		measure measurement; // Structure to keep the values we want to log
		struct timeval tv; // Structure to measure small time differences
		int index; // Which Arduino we are logging
		float diff_time; // Time since last measurement
		std::string::size_type sz;

		index = line.at(1) - 1;
		gettimeofday(&tv, NULL); // Saving the measurement time
		measurement.current_time = time(NULL);
		measurement.tv = tv;
		measurement.measured_illuminance = std::stof(line.substr(3), &sz);
		measurement.duty_cycle = std::stof(line.substr(3+sz), NULL);

		// We will check what's the current reference, to later calculate the error
		if(current_occupation[index] == 1)
			measurement.current_ref = current_ref_occupied[index];
		else
			measurement.current_ref = current_ref_unoccupied[index];

		elapsed_time = (measurement.tv.tv_sec - start.tv_sec) * 1000.0 + (measurement.tv.tv_usec - start.tv_usec)*0.001;

		std::cout << index + 1 << " " << measurement.measured_illuminance << " " << measurement.current_ref << " " << measurement.duty_cycle << " " << elapsed_time << std::endl;
		// Checks to see if we are streaming in real time any of the two parameters
		/*if(realtime[index*2 + 0]) // If we are streaming in real time the first parameter
			std::cout << "s l " << index + 1 << " " << measurement.measured_illuminance << " " << (float)measurement.tv.tv_usec;
		if(realtime[index*2 + 1]) // And the second one
			std::cout << "s d " << index + 1 << " " << measurement.duty_cycle << " " << (float)measurement.tv.tv_usec;*/

		if (bufferqueue[index].size() > 0) {
			diff_time = ((float)(measurement.tv.tv_usec - bufferqueue[index].back()->value().tv.tv_usec))*0.000001;
			// Time since last measurement calculated
			if (diff_time >= 0) // Error avoidance
				energy[index] += (measurement.duty_cycle/100)*diff_time; // Integral of the power
		}
		bufferqueue[index].push(measurement); // Uploading the measurement to the buffer
		if(bufferqueue[index].size() > perminute) // Buffer is longer than one minute
			bufferqueue[index].pop();
	}

	//tim.expires_from_now(boost::posix_time::milliseconds(1)); // Prepare new read
	//tim.async_wait(timer_handler); // Put read on the queue
	start_read_arduino();
}

void prettyPrint (std::string s){
	// Presents the extraordinary information sent by the Arduino
	char messagenum;
	int luminaire;
	float multifunfloat;

	luminaire = s.at(1);
	messagenum = s.at(3);
	multifunfloat = std::stof(s.substr(5), NULL);

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
			if(multifunfloat == 0){
				std::cout<< "ack" << std::endl;
			}
			else{
				std::cout << "err" << std::endl;
			}
			break;
		case set_occupied_value_ans:
			if(multifunfloat == 0){
				std::cout<< "ack" << std::endl;
			}
			else{
				std::cout << "err" << std::endl;
			}
			break;
		case set_unoccupied_value_ans:
			if(multifunfloat == 0){
				std::cout<< "ack" << std::endl;
			}
			else{
				std::cout << "err" << std::endl;
			}
			break;
		case set_cost_ans:
			if(multifunfloat == 0){
				std::cout<< "ack" << std::endl;
			}
			else{
				std::cout << "err" << std::endl;
			}
			break;
		case set_restart_ans:
			if(multifunfloat == 0){
				std::cout<< "ack" << std::endl;
			}
			else{
				std::cout << "err" << std::endl;
			}
			break;
	}
}

void start_read_input() {
	async_read_until(stm_desc, pccomms_buff, '\n', handle_read_input);
}

void handle_read_input (const error_code & ec, size_t len) {
	// Decodes the ASIO buffer into a string and forwards the string
	std::stringstream mybuffer;
	mybuffer.str(std::string()); //Clear buffer
	mybuffer << &pccomms_buff;
	std::string s = mybuffer.str();
	choose_case(s); // Sees what we want and handles the request
	start_read_input(); // We handled the input, we will prepare for another one
}

void choose_case(std::string s) {
	// Decodes the PC messages and forwards it to Arduino if needed
	// Presents the results of the requests
	int luminaire = 0;
	float multifunfloat;
	if(s.at(0) == 'g'){
		try {
			luminaire = std::stoi(s.substr(4));
		}
		catch(std::invalid_argument& e){
			luminaire = 0; // All luminaires
		}
		if(s.at(2) == 'l'){ // We just need to go to our buffer to see the last measurement
			std::cout << "l " << luminaire << " " << bufferqueue[luminaire-1].back()->value().measured_illuminance << std::endl;
		}
		else if(s.at(2) == 'd'){ // Same as above
			std::cout << "d " << luminaire << " " << bufferqueue[luminaire-1].back()->value().duty_cycle << std::endl;
		}
		else if(s.at(2) == 'o'){
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
			if(luminaire != 0){ // Last measurement's duty cycle is enough to calculate the power
				std::cout << "p " << luminaire << " " << bufferqueue[luminaire-1].back()->value().duty_cycle/100 << std::endl;
			}
			else{ // Case where we have to sum the three powers
				multifunfloat = 0;
				for(int i = 0; i < luminaireNumber; i++){
					multifunfloat += bufferqueue[i].back()->value().duty_cycle/100;
				}
				std::cout << "p T " << multifunfloat << std::endl;
			}
		}
		else if(s.at(2) == 't'){
			send_arduino(luminaire, 11);
		}
		else if(s.at(2) == 'e'){
			if(luminaire != 0){ // We just need to go to our energy vector, containing the integral
				std::cout << "e " << luminaire << " " << energy[luminaire-1] << std::endl;
			}
			else{ // Sum of all integrals
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
				for(Node *node=bufferqueue[luminaire-1].front(); node!=NULL; node=node->next()){
					// Sum of the difference between measured lux and the reference lux, when positive, at any given instant
					multifunfloat += std::max((float)0, node->value().measured_illuminance - node->value().current_ref);
					i++;
				}
				multifunfloat/= i; // Averaging the difference
				std::cout << "v " << luminaire << " " << multifunfloat << std::endl;
			}
			else{
				multifunfloat = 0;
				int i = 0;
				for (int j = 0; j < luminaireNumber; j++){ // Now summing for all the luminaires
					for(Node *node=bufferqueue[luminaire].front(); node!=NULL; node=node->next()){
						multifunfloat += std::max((float)0, node->value().measured_illuminance - node->value().current_ref);
						i++;
					}
				}
				multifunfloat /= (i/luminaireNumber); // And averaging taking into account that there are N luminaires
				std::cout << "v T " << multifunfloat << std::endl;
			}
		}
		else if(s.at(2) == 'f'){
			float diff_time;
			float lux1, lux2, lux3;
			if(luminaire != 0){
				multifunfloat = 0;
				int i = 0;
				for(Node *node=bufferqueue[luminaire-1].front(); node!=NULL; node=node->next()){
					// We will go through all values to calculate flicker error
					if(node->next() != NULL && node->next()->next() != NULL){
						// Checks if there are 2 values besides the node we are in
						lux1 = node->value().measured_illuminance;
						lux2 = node->next()->value().measured_illuminance;
						lux3 = node->next()->next()->value().measured_illuminance;
						if((lux1 - lux2) * (lux2 - lux3) < 0){
							diff_time = (node->next()->next()->value().tv.tv_usec - node->value().tv.tv_usec);
							// Time difference in microseconds
							if (diff_time > 0) {
								multifunfloat += (abs(lux1 - lux2) + abs(lux2 - lux3))/diff_time;
								i++;
							}
						}
					}
				}
				multifunfloat /= i; // Averaging the flicker error of all the collected samples
				std::cout << "f " << luminaire << " " << multifunfloat*1000000 << std::endl;
			}
			else{
				multifunfloat = 0;
				int i = 0;
				for(int j = 0; j < luminaireNumber; j++){
					// Same as before, but now we will average all the nodes
					for(Node *node=bufferqueue[luminaire].front(); node!=NULL; node=node->next()){
						if(node->next() != NULL && node->next()->next() != NULL){
							lux1 = node->value().measured_illuminance;
							lux2 = node->next()->value().measured_illuminance;
							lux3 = node->next()->next()->value().measured_illuminance;
							if((lux1 - lux2) * (lux2 - lux3) < 0){
							    diff_time = (node->next()->next()->value().tv.tv_usec - node->value().tv.tv_usec);
									// Difference also in microseconds
								if (diff_time > 0 ) {
									multifunfloat += (abs(lux1 - lux2) + abs(lux2 - lux3))/diff_time;
									i++;
								}
							}
						}
					}
				}
				multifunfloat /= (i/luminaireNumber); // Average taking into account there are N luminaires
				std::cout << "f T " << multifunfloat*1000000 << std::endl;
			}
		}
	}
	else if(s.at(0) == 'o'){
		luminaire = std::stoi(s.substr(2));
		multifunfloat = std::stoi(s.substr(4));
		// Besides sending the info to arduino, we also save it a vector, to help with statistics
		if(multifunfloat == 1){
			current_occupation[luminaire-1] = 1;
		}
		else{
			current_occupation[luminaire-1] = 0;
		}
		send_arduino(luminaire, set_occupied_ask, multifunfloat);
	}
	else if(s.at(0) == 'O'){
		luminaire = std::stoi(s.substr(2));
		multifunfloat = std::stof(s.substr(4));
		current_ref_occupied[luminaire-1] = multifunfloat;
		send_arduino(luminaire, set_occupied_value_ask, multifunfloat);
	}
	else if(s.at(0) == 'U'){
		luminaire = std::stoi(s.substr(2));
		multifunfloat = std::stof(s.substr(4));
		current_ref_unoccupied[luminaire-1] = multifunfloat;
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
		if(s.at(2) == 'l'){ // We just need to print the buffer
			luminaire = std::stoi(s.substr(4));
			printList(bufferqueue[luminaire-1], 1); // parameter 1
		}
		else if(s.at(2) == 'd'){
			luminaire = std::stoi(s.substr(4));
			printList(bufferqueue[luminaire-1], 2); // parameter 2
		}
	}
	else if(s.at(0) == 's'){
		if(s.at(2) == 'l'){ // We want to start or stop streaming
			luminaire = std::stoi(s.substr(4));
			realtime[(luminaire-1)*2 + 0] = !realtime[(luminaire-1)*2 + 0]; // Change bool status of parameter 0
		}
		else if(s.at(2) == 'd'){
			luminaire = std::stoi(s.substr(4));
			realtime[(luminaire-1)*2 + 1] = !realtime[(luminaire-1)*2 + 1]; // Change bool status of parameter 1
		}
	}
}

void send_arduino(char luminaire, char messagenum, float parameter){
	// Writes to Arduino in a predefined format
	std::ostringstream os;
	os << luminaire << ' ' << messagenum << ' ' << parameter << '\n';
	std::cout << os.str();

	async_write(sp, buffer(os.str()),
			[](const error_code &ec, size_t len){;}
				);
}

int main() {
	open_port();

	gettimeofday(&start, NULL);

	//program timer for read arduino operations
	//tim.expires_from_now(boost::posix_time::milliseconds(10));
	//tim.async_wait(timer_handler); // Start the reading arduino loop
	start_read_arduino();
	start_read_input(); // Input from the PC
	io.run(); //get things rolling
}
