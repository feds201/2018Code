/*
 * Pickup.cpp
 *
 *  Created on: Jan 26, 2018
 *      Author: feds201
 */

#include "Pickup.h"
#include "WPILib.h"

Pickup::Pickup(uint8_t PCM, int up, int down){

	solenoid = new DoubleSolenoid(PCM, up, down);

}

void Pickup::Toggle(){

	if(solenoid->Get() == frc::DoubleSolenoid::Value::kForward)
		solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
	else
		solenoid->Set(frc::DoubleSolenoid::Value::kForward);

}
