/*
 * Auton.cpp
 *
 *  Created on: Jan 16, 2018
 *      Author: feds201
 */
#include"Auton.h"

Auton::Auton(Drivetrain * drive){

	list = new struct AutonList;

	list->drive = drive;


}




