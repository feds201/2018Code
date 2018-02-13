/*
 * Pickup.h
 *
 *  Created on: Jan 26, 2018
 *      Author: feds201
 */

#ifndef SRC_PICKUP_H_
#define SRC_PICKUP_H_

#include "WPILib.h"
#include"ctre/Phoenix.h"

class Pickup{

	public:

	Pickup(uint8_t PCM, uint8_t m1id, uint8_t m2id, int up, int down, int in, int out);

	void Toggle();
	void Grab();
	void WheelSpeed(double speed);

	private:

	//Solenoid that controls pickup arm going up and down
	DoubleSolenoid *solenoid;

	//Solenoid that controls arm pinching
	DoubleSolenoid *sqSol;

	//Motors for wheels on arms
	WPI_TalonSRX *m1;
	WPI_TalonSRX *m2;

};



#endif /* SRC_PICKUP_H_ */
