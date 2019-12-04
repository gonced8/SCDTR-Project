// Implementation of hub related functions

#include "hub.h"


/*
aux = message.substring(2);
aux[1] = '\n';

if(aux.toInt() != 0){ // It is a letter
	second_char = aux[0];
	aux = message.substring(4);
}
else{ // It is a number -> message to specific node
	second_int = aux.toInt();
	aux = message.substring(4);
}
*/

/*-------Variable definition--------*/
String message;
String aux;
bool restart = false;
bool realtime[3] = {false, false, false};

/*--------Function definition--------*/
float getLux(){
	return 1.0;
}
float getPWM(){
	return 1.0;
}
float getOccupancy(){
	return 1.0;
}
float setOccupancy(){
	if(true)
		return 1.0;
	else
		return 0.0;
}
float getLowerBoundOccupied(){
	return 1.0;
}
float setLowerBoundOccupied(){
	if(true)
		return 1.0;
	else
		return 0.0;
}
float getLowerBoundUnoccupied(){
	return 1.0;
}
float setLowerBoundUnoccupied(){
	if(true)
		return 1.0;
	else
		return 0.0;
}
float getCurrentLowerBound(){
	return 1.0;
}
float getExternalIlluminace(){
	return 1.0;
}
float getIlluminaceControlRef(){
	return 1.0;
}
float getEnergyCost(){
	return 1.0;
}
float setEnergyCost(){
	if(true)
		return 1.0;
	else
		return 0.0;
}
float getPowerConsump(){
	return 1.0;
}
float getElapsedTime(){
	return 1.0;
}
float getAccumEnergy(){
	return 1.0;
}
float getAccumVis(){
	return 1.0;
}
float getAccumFlicker(){
	return 1.0;
}
float user_restart(){
	if(true)
		return 1.0;
	else
		return 0.0;
}

float ArduinoFunEncode(byte messagenum){
	if(messagenum == 1){
		return getLux();
	}
	else if(messagenum == 2){
		return getPWM();
	}
	else if(messagenum == 3){
		return getOccupancy();
	}
	else if(messagenum == 4){
		return setOccupancy();
	}
	else if(messagenum == 5){
		return getLowerBoundOccupied();
	}
	else if(messagenum == 6){
		return setLowerBoundOccupied();
	}
	else if(messagenum == 7){
		return getLowerBoundUnoccupied();
	}
	else if(messagenum == 8){
		return setLowerBoundUnoccupied();
	}
	else if(messagenum == 9){
		return getCurrentLowerBound();
	}
	else if(messagenum == 10){
		return getExternalIlluminace();
	}
	else if(messagenum == 11){
		return getIlluminaceControlRef();
	}
	else if(messagenum == 12){
		getEnergyCost();
	}
	else if(messagenum == 13){
		return setEnergyCost();
	}
	else if(messagenum == 14){
		return getPowerConsump();
	}
	else if(messagenum == 15){
		getElapsedTime();
	}
	else if(messagenum == 16){
		return getAccumEnergy();
	}
	else if(messagenum == 17){
		return getAccumVis();
	}
	else if(messagenum == 18){
		getAccumFlicker();
	}
	else if(messagenum == 19){
		return user_restart();
	}
}

void SerialDecode(){
	byte messagenum = 0;
	message = Serial.readString();
	int luminaire;
	int multifunint;
	float multifunfloat;
	if(message[0] == 'g'){
		if(message[2] == 'l'){
			luminaire = (message.substring(4)).toInt();
			messagenum = 1;

		}
		else if(message[2] == 'd'){
			luminaire = (message.substring(4)).toInt();
			messagenum = 2;
		}
		else if(message[2] == 'o'){
			luminaire = (message.substring(4)).toInt();
			messagenum = 3;
		}
		else if(message[2] == 'O'){
			luminaire = (message.substring(4)).toInt();
			messagenum = 4;
		}
		else if(message[2] == 'U'){
			luminaire = (message.substring(4)).toInt();
			messagenum = 5;
		}
		else if(message[2] == 'L'){
			luminaire = (message.substring(4)).toInt();
			messagenum = 6;
		}
		else if(message[2] == 'x'){
			luminaire = (message.substring(4)).toInt();
			messagenum = 7;
		}
		else if(message[2] == 'r'){
			luminaire = (message.substring(4)).toInt();
			messagenum = 8;
		}
		else if(message[2] == 'c'){
			luminaire = (message.substring(4)).toInt();
			messagenum = 9;
		}
		else if(message[2] == 'p'){
			luminaire = (message.substring(4)).toInt();
			messagenum = 10;
			// if message g p T instead of g p <i>, luminaire will be 0
		}
		else if(message[2] == 't'){
			luminaire = (message.substring(4)).toInt();
			messagenum = 11;
		}
		else if(message[2] == 'e'){
			luminaire = (message.substring(4)).toInt();
			messagenum = 12;
			// if message g e T instead of g e <i>, luminaire will be 0
		}
		else if(message[2] == 'v'){
			luminaire = (message.substring(4)).toInt();
			messagenum = 13;
			// if message g v T instead of g v <i>, luminaire will be 0
		}
		else if(message[2] == 'f'){
			luminaire = (message.substring(4)).toInt();
			messagenum = 14;
			// if message g f T instead of g f <i>, luminaire will be 0
		}
	}
	else if(message[0] == 'o'){
		luminaire = message.substring(2).toInt();
		multifunint = message.substring(4).toInt();
		messagenum = 15;
	}
	else if(message[0] == 'O'){
		luminaire = message.substring(2).toInt();
		multifunfloat = message.substring(4).toFloat();
		messagenum = 16;
	}
	else if(message[0] == 'U'){
		luminaire = message.substring(2).toInt();
		multifunfloat = message.substring(4).toFloat();
		messagenum = 17;
	}
	else if(message[0] == 'c'){
		luminaire = message.substring(2).toInt();
		multifunfloat = message.substring(4).toFloat();
		messagenum = 18;
	}
	else if(message[0] == 'r'){
		restart = true;
		messagenum = 19;
	}
	else if(message[0] == 'b'){
		if(message[2] == 'l'){
			luminaire = (message.substring(4)).toInt();
			messagenum = 20;
		}
		else if(message[2] == 'd'){
			luminaire = (message.substring(4).toInt());
			messagenum = 21;
		}
	}
	else if(message[0] == 's'){
		if(message[2] == 'l'){
			luminaire = (message.substring(4)).toInt();
			messagenum = 22;
		}
		else if(message[2] == 'd'){
			luminaire = (message.substring(4).toInt());
			messagenum = 23;
		}
		realtime[luminaire-1] = ~realtime[luminaire-1];
	}
}

void SerialEncode(){
	float buffer[20];
	byte messagenum;
	int luminaire;
	float value;
	//memcpy(&value, message+1)
	if(messagenum == 1){
		Serial.print("l ");
		Serial.print(luminaire);
		Serial.print(" ");
		Serial.print(value);
	}
	else if(messagenum == 2){
		Serial.print("d ");
		Serial.print(luminaire);
		Serial.print(" ");
		Serial.print(value);
	}
	else if(messagenum == 3){
		Serial.print("o ");
		Serial.print(luminaire);
		Serial.print(" ");
		Serial.print(value);
	}
	else if(messagenum == 4){
		if(value == 1)
			Serial.print("ack");
		else
			Serial.print("err");
	}
	else if(messagenum == 5){
		Serial.print("O ");
		Serial.print(luminaire);
		Serial.print(" ");
		Serial.print(value);
	}
	else if(messagenum == 6){
		if(value == 1)
			Serial.print("ack");
		else
			Serial.print("err");
	}
	else if(messagenum == 7){
		Serial.print("U ");
		Serial.print(luminaire);
		Serial.print(" ");
		Serial.print(value);
	}
	else if(messagenum == 8){
		if(value == 1)
			Serial.print("ack");
		else
			Serial.print("err");
	}
	else if(messagenum == 9){
		Serial.print("L ");
		Serial.print(luminaire);
		Serial.print(" ");
		Serial.print(value);
	}
	else if(messagenum == 10){
		Serial.print("x ");
		Serial.print(luminaire);
		Serial.print(" ");
		Serial.print(value);
	}
	else if(messagenum == 11){
		Serial.print("r ");
		Serial.print(luminaire);
		Serial.print(" ");
		Serial.print(value);
	}
	else if(messagenum == 12){
		Serial.print("c ");
		Serial.print(luminaire);
		Serial.print(" ");
		Serial.print(value);
	}
	else if(messagenum == 13){
		if(value == 1)
			Serial.print("ack");
		else
			Serial.print("err");
	}
	else if(messagenum == 14){
		Serial.print("p ");
		if(luminaire != 0)
			Serial.print(luminaire);
		else
			Serial.print("T");
		Serial.print(" ");
		Serial.print(value);
	}
	else if(messagenum == 15){
		Serial.print("t ");
		Serial.print(luminaire);
		Serial.print(" ");
		Serial.print(value);
	}
	else if(messagenum == 16){
		Serial.print("e ");
		if(luminaire != 0)
			Serial.print(luminaire);
		else
			Serial.print("T");
			// In this case value has to be the same of the three floats!!!
		Serial.print(" ");
		Serial.print(value);
	}
	else if(messagenum == 17){
		Serial.print("v ");
		if(luminaire != 0)
			Serial.print(luminaire);
		else
			Serial.print("T");
		Serial.print(" ");
		Serial.print(value);
	}
	else if(messagenum == 18){
		Serial.print("f ");
		if(luminaire != 0)
			Serial.print(luminaire);
		else
			Serial.print("T");
		Serial.print(" ");
		Serial.print(value);
	}
	else if(messagenum == 19){
		if(value == 1)
			Serial.print("ack");
		else
			Serial.print("err");
	}
}
