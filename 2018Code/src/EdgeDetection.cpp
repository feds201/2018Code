/*
 * EdgeDetection.cpp
 *
 *  Created on: Jan 12, 2017
 *      Author: Andy
 */

#include <EdgeDetection.h>

//Class to check if a button is being pressed, not held down

//Uses initializer list to initialize last time and this time to inital parameter
Edge::Edge(bool inital) :
lastTime(inital),
thisTime(inital)
{
}

//Update the state variables. lastTime= what is was before, thisTime= new state
void Edge::update(bool state)
{
	lastTime=thisTime;
	thisTime = state;
}

//Check if a button is pressed. If button is down at the moment and it wasn't down last time, it is pressed.
bool Edge::isPressed()
{
	return (thisTime == true && lastTime == false);

}

//Not used
bool Edge::getState(){

	return thisTime;

}


