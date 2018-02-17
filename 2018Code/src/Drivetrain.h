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

		double P = 0;
		double I = 0;
		double D = 0;
		double F = 0.1023;

		int maxSpeed = 10000;

		double gearRatioHi = (40/34);
		double gearRatioLo = (60/14)/0.74;
		double wheelR = 3;
		double encCountsPerRev = 4096;
		double pi = 3.14159;
		double countsPerInHi;
		double countsPerInLo;
		double maxV = 10000;
		float inchesPerMeter = 39.37007874;
		double metersPerCountLo;
		double metersPerCountHi;
		double maxVHi;
		double maxVLo;
		float accelTimeHi;
		float accelTimeLo;

		bool HiGear = false;


		WPI_TalonSRX *Left1;
		WPI_TalonSRX *Left2;
		WPI_TalonSRX *Right1;
		WPI_TalonSRX *Right2;

		uint8_t L2ID;
		uint8_t R2ID;

		DoubleSolenoid *shifter;

		Preferences *prefs;

		PigeonIMU *gyro;

		int16_t ba_xyz[3];

		double accelX, accelY, accelZ;

		BuiltInAccelerometer* accel;

		int MAXAccel = 0.82;

		double leftSet;
		double rightSet;

		bool autoHeading = true;

		int pcm;
		int shifterFWD;
		int shifterREV;

		int EncPos[2];
		int EncVel[2];

	};

	struct driveList* list;



};





#endif /* SRC_DRIVETRAIN_H_ */
