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

Pickup::Pickup(uint8_t PCM, uint8_t m1id, uint8_t m2id, int up, int down, int in, int out){

	solenoid = new DoubleSolenoid(PCM, up, down);
	sqSol = new DoubleSolenoid(PCM, in , out);

	m1 = new WPI_TalonSRX(m1id);
	m2 = new WPI_TalonSRX(m2id);

	solenoid->Set(frc::DoubleSolenoid::Value::kForward);
	sqSol->Set(frc::DoubleSolenoid::Value::kForward);

	std::cout << "Pickup Init" << std::endl;

}

void Pickup::Toggle(){

	if(solenoid->Get() == frc::DoubleSolenoid::Value::kForward)
		solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
	else
		solenoid->Set(frc::DoubleSolenoid::Value::kForward);

}

void Pickup::Grab(){

	if(sqSol->Get() == frc::DoubleSolenoid::Value::kForward)
		sqSol->Set(frc::DoubleSolenoid::Value::kReverse);
	else
		sqSol->Set(frc::DoubleSolenoid::Value::kForward);

}


void Pickup::WheelSpeed(double speed){

	m1->Set(speed);
	m2->Set(-speed);

}
