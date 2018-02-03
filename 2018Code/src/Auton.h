/*
 * Auton.h
 *
 *  Created on: Jan 16, 2018
 *      Author: feds201
 */

#ifndef SRC_AUTON_H_
#define SRC_AUTON_H_

#include"WPILib.h"
#include"ctre/Phoenix.h"
#include"Drivetrain.h"
#include"Timer.h"


class Auton{

public:

Auton(Drivetrain* drive, SampleRobot* robot);
void Drive(double speed, double dist);
void Rotate(double angle);
void Arc(double speed, double turnAngle, double radius);




private:

	struct AutonList{

		Drivetrain* drive;

		Timer timer;

		frc::SampleRobot *robot;

		bool hasTurned = false;
		bool offSet = 0;

		double gearRatioHi;
		double gearRatioLo;
		double wheelR;
		double encCountsPerRev;
		double pi = 3.14159;
		double Track;

		double ViSetPt;
		double VoSetPt;

		double counterPerIn = 1950;
		double Ptheta = 0;
		double Pv = 0;

		double Wc = 0;
		double Vc = 0;
		double Rc = 0;
		double thetac = 0;
		double thetaErr = 0;
		double Vi = 0;
		double Vo = 0;
		double Si = 0;
		double So = 0;
		double Ra = 0;
		double Va = 0;
		double VErr = 0;
		double t = 0;
		double theta = 0;

	};

	struct AutonList *list;

};





#endif /* SRC_AUTON_H_ */
