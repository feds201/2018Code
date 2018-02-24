/*
 * Pickup.cpp
 *
 *  Created on: Jan 26, 2018
 *      Author: feds201
 */

#include "Pickup.h"
#include "WPILib.h"
#include"ctre/Phoenix.h"
#include<iostream>

//Class for controlling arms and everything associated

//Solenoids control air
//PCM controls solenoids

Pickup::Pickup(uint8_t PCM, uint8_t m1id, uint8_t m2id, int up, int down, int in, int out){

	//Create DoubleSolenoid object, controls arm going up and down. PCM- canID for pneumatic control module, up- ports on the PCM that make arm go up, down is opposite
	solenoid = new DoubleSolenoid(PCM, up, down);
	sqSol = new DoubleSolenoid(PCM, in , out);

	//Makes new motors, m1id & m2id are canIDs of CANTALONS
	m1 = new WPI_TalonSRX(m1id);
	m2 = new WPI_TalonSRX(m2id);

	//kForward makes arm go up (generally just makes the object do ON)
	solenoid->Set(frc::DoubleSolenoid::Value::kForward);

	//kForward makes arms pinch
	sqSol->Set(frc::DoubleSolenoid::Value::kForward);

	//Prints to driver station
	std::cout << "Pickup Init" << std::endl;

}

//Makes arms go up or down, reverse of what it already is
void Pickup::Toggle(){

	//If arm up, go down
	if(solenoid->Get() == frc::DoubleSolenoid::Value::kForward)
		solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
	else //(Arm is already down), Go up
		solenoid->Set(frc::DoubleSolenoid::Value::kForward);

}

//Makes pinchers go in or out, reverse of what it already is
void Pickup::Grab(){

	//If pinchers forward, retract
	if(sqSol->Get() == frc::DoubleSolenoid::Value::kForward)
		sqSol->Set(frc::DoubleSolenoid::Value::kReverse);
	else //(Pincher is reverse), Go forward
		sqSol->Set(frc::DoubleSolenoid::Value::kForward);

}

//Sets speed of wheels on pinchers, might not need the parameter, only there for now because Andy doesn't know what the appropriate speed is yet
void Pickup::WheelSpeed(double speed){

	m1->Set(speed);
	m2->Set(-speed);

}
