/*
 * Elevator.cpp
 *
 *  Created on: Jan 24, 2018
 *      Author: feds201
 */


#include"Elevator.h"

Elevator::Elevator(uint8_t motorID, int PCM, int fwdsolenoid, int revsolenoid, int tlimit, int mlimit, int blimit){

	list = new struct EList;
	list->motor = new WPI_TalonSRX(motorID);
	list->solenoid = new DoubleSolenoid(PCM, fwdsolenoid, revsolenoid);
	list->toplimit = new DigitalInput(tlimit);
	list->middlelimit = new DigitalInput(mlimit);
	list->bottomlimit = new DigitalInput(blimit);

}

void Elevator::Move(double speed){

	if(speed < 0 && list->bottomlimit->Get()){
		list->motor->Set(speed);
	}else if(speed > 0 && list->toplimit->Get()){
		list->motor->Set(speed);
	}else{
		list->motor->Set(0);
	}

}

void Elevator::Bottom(){

	if(!list->top && !list->middle && !list->bottom)
		list->bottom = true;

}

void Elevator::Middle(){

	if(!list->top && !list->middle && !list->bottom)
		list->middle = true;

}

void Elevator::Top(){

	if(!list->top && !list->middle && !list->bottom)
		list->top = true;

}

void Elevator::Refresh(){

	if(list->toplimit->Get())
		list->top = false;
	else if(list->middlelimit->Get()){
		list->middle = false;
		list->wasbottom = false;
		list->wastop = false;
	}else if(list->bottomlimit->Get())
		list->bottom = false;

	if(list->middle){
		if(list->bottomlimit->Get()){
			list->wasbottom = true;
		}else if(list->toplimit->Get()){
			list->wastop = true;
		}else if(list->wasbottom){
			Move(.35);
		}else if(list->wastop){
			Move(-.35);
		}else{
			Move(-.35);
		}

	}else if(list->top)
		Move(.35);
	else if(list->bottom)
		Move(-.35);

}
