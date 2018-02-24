#include <Drivetrain.h>
#include"WPILib.h"
#include"ctre/Phoenix.h"
#include<iostream>
#include"Math.h"
#include"Elevator.h"

Drivetrain::Drivetrain(uint8_t L1, uint8_t L2, uint8_t R1, uint8_t R2, uint8_t gyro, int PCM, int shifterFWD, int shifterREV, Elevator* ele){

	list = new struct driveList;

	//Initialize gyro object with CANID
	list->gyro = new PigeonIMU(gyro);

	//Sets gyro to a angle of 0 with a 10 millisecond timeout (doesn't really matter)
	list->gyro->SetFusedHeading(0, 10);

	// vv
	list->prefs = Preferences::GetInstance();

	//list->accel = new BuiltInAccelerometer(Accelerometer::kRange_8G);

	//Initialize motors with canIDs
	list->Left1 = new WPI_TalonSRX(L1);
	list->Left2 = new WPI_TalonSRX(L2);
	list->Right1 = new WPI_TalonSRX(R1);
	list->Right2 = new WPI_TalonSRX(R2);

	//Assigns vars to appropriate CANID
	list->L2ID = L2;
	list->R2ID = R2;

	//Makes new DoubleSolenoid, PCM is CANID of pneumatic control module, shifterFWD and REV are ports on PCM
	list->shifter = new DoubleSolenoid(PCM, shifterFWD, shifterREV);

	//Assigns associated values to list struct
	list->pcm = PCM;
	list->shifterFWD = shifterFWD;
	list->shifterREV = shifterREV;

	list->countsPerInLo = list->encCountsPerRev*(list->gearRatioLo*(1/(2*list->pi*list->wheelR)));
	list->countsPerInHi = list->encCountsPerRev*(list->gearRatioHi*(1/(2*list->pi*list->wheelR)));

	list->metersPerCountHi = 1/(list->countsPerInHi*list->inchesPerMeter);
	list->metersPerCountLo = 1/(list->countsPerInLo*list->inchesPerMeter);

	list->metersPerCountHi *= list->maxSpeed;
	list->metersPerCountLo *= list->maxSpeed;

	list->maxVHi = list->metersPerCountHi*10;
	list->maxVLo = list->metersPerCountLo*10;

	list->accelTimeHi = list->maxVHi/list->MaxAccelHi;
	list->accelTimeLo = list->maxVHi/list->MAXAccel;

	list->accelHiUp = list->maxVHi/list->MaxAccelHi;
	list->accelLoUp = list->maxVLo/list->MAxAccelUp;

	list->mHi = (list->accelHiUp-list->accelTimeHi)/35270;
	list->mLo = (list->accelLoUp-list->accelTimeLo)/35270;

	list->Left2->ConfigClosedloopRamp(list->accelTimeLo, 10);
	list->Right2->ConfigClosedloopRamp(list->accelTimeLo, 10);

	list->Left1->Set(ControlMode::PercentOutput, 0);
	list->Left2->Set(ControlMode::PercentOutput, 0);
	list->Right1->Set(ControlMode::PercentOutput, 0);
	list->Right2->Set(ControlMode::PercentOutput, 0);

	//Second parameter is timeout in ms
	list->Left2->ConfigNominalOutputForward(0, 10);
	list->Right2->ConfigNominalOutputForward(0, 10);
	list->Left2->ConfigNominalOutputReverse(0, 10);
	list->Right2->ConfigNominalOutputReverse(0, 10);

	//Puts values on shuffleboard screen
	list->prefs->PutDouble("P", list->P);
	list->prefs->PutDouble("I", list->I);
	list->prefs->PutDouble("D", list->D);
	list->prefs->PutDouble("F", list->F);

	//Left2 and Right2 are motors that have encoders, vv
	list->Left2->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);
	list->Right2->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);

	list->Left2->Config_kP(0, list->P, 10);
	list->Right2->Config_kP(0, list->P, 10);
	list->Left2->Config_kI(0, list->I, 10);
	list->Right2->Config_kI(0, list->I, 10);
	list->Left2->Config_kD(0, list->D, 10);
	list->Right2->Config_kD(0, list->D, 10);
	list->Left2->Config_kF(0, list->F, 10);
	list->Right2->Config_kF(0, list->F, 10);

	//Set robot gearbox to low gear
	list->shifter->Set(frc::DoubleSolenoid::Value::kForward);

	std::cout << "Drivetrain Done" << std::endl;

	list->currTime = list->accelTimeLo;
	list->lastTime = list->currTime;

}

void Drivetrain::Drive(float fwd, float trn, bool autoHeading){

	//
	SmartDashboard::PutNumber("Gyro", getGyroAngle());
	SmartDashboard::PutNumber("LEnc", GetEncPos()[0]);
	SmartDashboard::PutNumber("REnc", GetEncPos()[1]);

	//list->gyro->GetBiasedAccelerometer(list->ba_xyz);

/*

	if(list->HiGear){

		list->currTime = (list->mHi*list->ele->getHeight())+list->accelTimeHi;

	}else{

		list->currTime = (list->mLo*list->ele->getHeight())+list->accelTimeLo;

	}
	*/
/*
	if(list->lastTime != list->currTime){
		list->Left2->ConfigClosedloopRamp(list->currTime, 10);
		list->Right2->ConfigClosedloopRamp(list->currTime, 10);
		list->lastTime = list->currTime;
	}
*/
	//list->accelX = list->ba_xyz[0]/16384;
	//list->accelY = list->ba_xyz[1]/16384;
	//list->accelZ = list->ba_xyz[2]/16384;

	/*
	SmartDashboard::PutNumber("AccelX", list->accelX);
	SmartDashboard::PutNumber("AccelY", list->accelY);
	SmartDashboard::PutNumber("AccelZ", list->accelZ);
	SmartDashboard::PutNumber("RAWAccelX", list->ba_xyz[0]);
	SmartDashboard::PutNumber("RAWAccelY", list->ba_xyz[1]);
	SmartDashboard::PutNumber("RAWAccelX", list->ba_xyz[2]);
	SmartDashboard::PutNumber("AccelX_RIO", list->accel->GetX());
	SmartDashboard::PutNumber("AccelY_RIO", list->accel->GetY());
	SmartDashboard::PutNumber("AccelZ_RIO", list->accel->GetZ());
*/
	//fwd *= 1-pow((list->accelX/list->MAXAccel), 1);

	if(trn != 0 && autoHeading)
		list->gyro->SetFusedHeading(0, 10);

	if(autoHeading){
		list->leftSet = list->maxSpeed*((trn+(list->gyro->GetFusedHeading()*0.07)) - fwd);
		list->rightSet = list->maxSpeed*((trn+(list->gyro->GetFusedHeading()*0.07)) + fwd);
	}else{
		list->leftSet = list->maxSpeed*(trn-fwd);
		list->rightSet = list->maxSpeed*(trn+fwd);
	}

	if(list->P != list->prefs->GetDouble("P") or list->I != list->prefs->GetDouble("I") or list->D != list->prefs->GetDouble("D") or list->F != list->prefs->GetDouble("F")){
		list->P = list->prefs->GetDouble("P");
		list->I = list->prefs->GetDouble("I");
		list->D = list->prefs->GetDouble("D");
		list->F = list->prefs->GetDouble("F");

		list->Left2->Config_kP(0, list->P, 10);
		list->Right2->Config_kP(0, list->P, 10);
		list->Left2->Config_kI(0, list->I, 10);
		list->Right2->Config_kI(0, list->I, 10);
		list->Left2->Config_kD(0, list->D, 10);
		list->Right2->Config_kD(0, list->D, 10);
		list->Left2->Config_kF(0, list->F, 10);
		list->Right2->Config_kF(0, list->F, 10);
	}

	Set(list->leftSet, list->rightSet);



}

void Drivetrain::Set(float Left, float Right){

	list->Left2->Set(ControlMode::Velocity, Left);
	list->Right2->Set(ControlMode::Velocity, Right);
	list->Left1->Set(ControlMode::Follower, list->L2ID);
	list->Right1->Set(ControlMode::Follower, list->R2ID);

}

//Shift the gear of the robot
void Drivetrain::Shift(){

	//If robot is in low gear, switch to high and set HiGear to true
	if(list->shifter->Get() == frc::DoubleSolenoid::Value::kForward){
			list->shifter->Set(frc::DoubleSolenoid::Value::kReverse);
			list->HiGear = true;

			//Configure ramping times on motors to suit High Gear
			list->Left2->ConfigClosedloopRamp(list->accelTimeHi, 10);
			list->Right2->ConfigClosedloopRamp(list->accelTimeHi, 10);
		}

	//If robot is in high gear set it to low and set HiGeat to false
	else{
		list->shifter->Set(frc::DoubleSolenoid::Value::kForward);
		list->HiGear = false;

		//Configure ramping times on motors to suit Low Gear
		list->Left2->ConfigClosedloopRamp(list->accelTimeLo, 10);
		list->Right2->ConfigClosedloopRamp(list->accelTimeLo, 10);
	}
}

int * Drivetrain::GetEncVel(){

	list->EncVel[0] = list->Left2->GetSelectedSensorVelocity(0);
	list->EncVel[1] = list->Right2->GetSelectedSensorVelocity(0);

	return list->EncVel;

}

int * Drivetrain::GetEncPos(){

	list->EncPos[0] = list->Left2->GetSelectedSensorPosition(0);
	list->EncPos[1] = list->Right2->GetSelectedSensorPosition(0);

	return list->EncPos;

}

void Drivetrain::SetEncPos(double left, double right){

	list->Left2->SetSelectedSensorPosition(left, 0, 10);
	list->Right2->SetSelectedSensorPosition(right, 0, 10);

}

//Get angle of PigeonIMU gyro
double Drivetrain::getGyroAngle(){
	return list->gyro->GetFusedHeading();
}

//Set angle of gyro, second parameter of SetFusedHeading is timeout in millseconds (not important)
void Drivetrain::setGyroAngle(double angle){
	list->gyro->SetFusedHeading(angle, 10);
}

float * Drivetrain::GetCurr(){

	list->current[0] = list->Left2->GetOutputCurrent();
	list->current[1] = list->Right2->GetOutputCurrent();

	return list->current;
}
