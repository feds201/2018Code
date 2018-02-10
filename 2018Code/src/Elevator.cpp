/*
 * Elevator.cpp
 *
 *  Created on: Jan 24, 2018
 *      Author: feds201
 */


#include"Elevator.h"
#include<iostream>

Elevator::Elevator(uint8_t motorID, int PCM, int fwdsolenoid, int revsolenoid, int presToggleHi, int presToggleHiOff, int presToggleLo,  int tlimit, int blimit){

	//Test Again
	list = new struct EList;
	list->motor = new WPI_TalonSRX(motorID);
	list->solenoid = new DoubleSolenoid(PCM, fwdsolenoid, revsolenoid);
	list->hiPresToggle = new DoubleSolenoid(PCM, presToggleHi, presToggleHiOff);
	list->loPresToggle = new Solenoid(PCM, presToggleLo);
	list->toplimit = new DigitalInput(tlimit);
	list->bottomlimit = new DigitalInput(blimit);

	list->motor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);

	list->hiPresToggle->Set(frc::DoubleSolenoid::Value::kReverse);
	list->solenoid->Set(frc::DoubleSolenoid::Value::kReverse);

	std::cout << "Ele Init" << std::endl;

}

void Elevator::Move(double speed){

	if(speed != 0 && speed < 0 && list->motor->GetSelectedSensorPosition(0) < 2000){
		speed = -0.2;
	}else if(speed != 0 && speed > 0 && list->motor->GetSelectedSensorPosition(0) > 10000){
		speed = 0.2;
	}


	if(speed < 0 && list->bottomlimit->Get()){
		list->motor->Set(speed);
	}else if(speed > 0 && list->toplimit->Get()){
		list->motor->Set(speed);
	}else{
		list->motor->Set(0);
	}

}


void Elevator::PushHi(){

	if(list->solenoid->Get() == frc::DoubleSolenoid::Value::kForward){
		list->hiPresToggle->Set(frc::DoubleSolenoid::Value::kReverse);
		list->loPresToggle->Set(true);
		list->solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
	}else{
		list->hiPresToggle->Set(frc::DoubleSolenoid::Value::kForward);
		list->loPresToggle->Set(false);
		list->solenoid->Set(frc::DoubleSolenoid::Value::kForward);
	}


}

void Elevator::PushLo(){

	if(list->solenoid->Get() == frc::DoubleSolenoid::Value::kForward){
		list->hiPresToggle->Set(frc::DoubleSolenoid::Value::kReverse);
		list->loPresToggle->Set(true);
		list->solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
	}else{
		list->hiPresToggle->Set(frc::DoubleSolenoid::Value::kReverse);
		list->loPresToggle->Set(true);
		list->solenoid->Set(frc::DoubleSolenoid::Value::kForward);
	}



}

