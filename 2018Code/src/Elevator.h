/*
 * Elevator.h
 *
 *  Created on: Jan 24, 2018
 *      Author: feds201
 */

#ifndef SRC_ELEVATOR_H_
#define SRC_ELEVATOR_H_
#include "WPILib.h"
#include "ctre/Phoenix.h"

class Elevator{

public:
	Elevator(uint8_t motorID, int PCM, int fwdsolenoid, int revsolenoid, int presToggleHi, int presToggleHiOff, int presToggleLo, int tlimit, int blimit);
	void Move(double speed);
	void Top();
	void Bottom();
	void Middle();
	void PushHi();
	void PushLo();
	void Refresh();

private:

	struct EList{

		//Motor that controls elevator
		WPI_TalonSRX *motor;

		//Controls piston that pushes cube
		DoubleSolenoid *solenoid;

		//Toggles flow of high pressure for cube ejector
		DoubleSolenoid *hiPresToggle;

		//Toggles flow of low pressure for cube ejector
		Solenoid *loPresToggle;

		//Switch on top indicating if elevator is touching it
		DigitalInput *toplimit;
		DigitalInput *middlelimit;
		DigitalInput *bottomlimit;

		//Not using anymore
		bool top = false;
		bool middle = false;
		bool bottom = false;
		bool wastop = false;
		bool wasbottom = false;

	};

	//Declares pointer list to struct Elist
	struct EList* list;

};

#endif /* SRC_ELEVATOR_H_ */
