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

		WPI_TalonSRX *motor;
		DoubleSolenoid *solenoid;
		DoubleSolenoid *hiPresToggle;
		Solenoid *loPresToggle;
		DigitalInput *toplimit;
		DigitalInput *middlelimit;
		DigitalInput *bottomlimit;
		bool top = false;
		bool middle = false;
		bool bottom = false;
		bool wastop = false;
		bool wasbottom = false;

	};

	struct EList* list;

};

#endif /* SRC_ELEVATOR_H_ */
