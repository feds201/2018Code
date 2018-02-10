/*
 * EdgeDetection.h
 *
 *  Created on: Jan 12, 2017
 *      Author: Andy
 */

#ifndef SRC_EDGEDETECTION_H_
#define SRC_EDGEDETECTION_H_

//Class to check if a button is being pressed, not held down
class Edge {
public:
	Edge(bool inital=false);
	void update(bool state);

	bool isPressed();
	bool getState();

private:

	//If the button was pressed last time
	bool lastTime;

	//If the button was pressed this time
	bool thisTime;
};

#endif /* SRC_EDGEDETECTION_H_ */
