/*
 * Drivetrain.h
 *
 *  Created on: Jan 16, 2018
 *      Author: feds201
 */

#ifndef SRC_DRIVETRAIN_H_
#define SRC_DRIVETRAIN_H_

#include"WPILib.h"
#include"ctre/Phoenix.h"

class Drivetrain{

public:

	Drivetrain(uint8_t L1, uint8_t L2, uint8_t R1, uint8_t R2, uint8_t gyro, int PCM, int shifterFWD, int shifterREV);
	void Drive(float fwd, float trn, bool autoHeading);
	void directSet(float left, float right);
	void Set(float Left, float Right);
	void Shift();
	int * GetEncVel();
	int * GetEncPos();
	void SetEncPos(double left, double right);
	double getGyroAngle();
	void setGyroAngle(double angle);

private:

	struct driveList{

		// vv
		double P = 0;
		double I = 0;
		double D = 0;
		double F = 0.1023;

		//Maxmimum amount of counts encoders can go per 100ms
		int maxSpeed = 10000;

		//Motors for wheels
		WPI_TalonSRX *Left1;
		WPI_TalonSRX *Left2;
		WPI_TalonSRX *Right1;
		WPI_TalonSRX *Right2;

		//CANID for left motor and right motor (ones with encoders on them), not sure if used
		uint8_t L2ID;
		uint8_t R2ID;

		//Solenoid that controls shifters that control gearboxes
		DoubleSolenoid *shifter;

		//Creates Preferences object (display on dashboard)
		Preferences *prefs;

		//Gyro on robot
		PigeonIMU *gyro;

		//Target speed for the motors
		double leftSet;
		double rightSet;

		//Decides whether or not gyro gets used in drivetrain
		bool autoHeading = true;

		//CANID for PCM
		int pcm;

		//Note: On drivetrain kForward is low gear and kReverse is high gear

		//FWD- port on PCM that shifts solenoid forward, REV is opposite
		int shifterFWD;
		int shifterREV;

		//
		int EncPos[2];
		int EncVel[2];

	};

	struct driveList* list;



};





#endif /* SRC_DRIVETRAIN_H_ */
