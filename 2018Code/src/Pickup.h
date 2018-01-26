/*
 * Pickup.h
 *
 *  Created on: Jan 26, 2018
 *      Author: feds201
 */

#ifndef SRC_PICKUP_H_
#define SRC_PICKUP_H_

#include "WPILib.h"

class Pickup{

	public:

	Pickup(uint8_t PCM, int up, int down);

	void Toggle();

	private:

	DoubleSolenoid *solenoid;

};



#endif /* SRC_PICKUP_H_ */
