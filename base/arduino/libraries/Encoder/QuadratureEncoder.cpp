#include "QuadratureEncoder.h"


Encoder::Encoder(){
	greenCablePin = 23;
	yellowCablePin = 22;
}

Encoder::Encoder(WheelFace a){
	faceOfWheel = a;

}

Encoder::Encoder(WheelFace a,int green,int yellow){
  faceOfWheel = a;
  greenCablePin = green;
  yellowCablePin = yellow;

}


void Encoder::attach(int green,int yellow){
  greenCablePin = green;
  yellowCablePin = yellow;

}


void Encoder::initialize(){
  encoderTicks = 0;
  pinMode(greenCablePin,INPUT_PULLUP);
  // digitalWrite(greenCablePin,LOW);
  pinMode(yellowCablePin,INPUT_PULLUP);
  // digitalWrite(yellowCablePin,LOW);

}


void Encoder::handleInterruptGreen(){
	_greenSet = digitalRead(greenCablePin);
	_yellowSet = digitalRead(yellowCablePin);
	encoderTicks += parseEncoder();
	_greenPrev = _greenSet;
	_yellowPrev = _yellowSet;
}


void Encoder::handleInterruptYellow(){
	_yellowSet = digitalRead(yellowCablePin);
	_greenSet = digitalRead(greenCablePin);
	encoderTicks += parseEncoder();
	_yellowPrev = _yellowSet;
	_greenPrev = _greenSet;

}

inline int Encoder::parseEncoder(){
	int result;
	if(_greenPrev && _yellowPrev) {
		if(!_greenSet && _yellowSet) result = -1;
		if(_greenSet && !_yellowSet) result = 1;
	}
	else if(!_greenPrev && _yellowPrev) {
		if(!_greenSet && !_yellowSet) result = -1;
		if(_greenSet && _yellowSet) result = 1;
	}
	else if(!_greenPrev && !_yellowPrev) {
		if(_greenSet && !_yellowSet) result = -1;
		if(!_greenSet && _yellowSet) result = 1;
	}
	else if(_greenPrev && !_yellowPrev) {
		if(_greenSet && _yellowSet) result = -1;
		if(!_greenSet && !_yellowSet) result = 1;
	}
	return faceOfWheel * result;
}
