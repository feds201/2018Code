#include <Drivetrain.h>
#include"WPILib.h"
#include"ctre/Phoenix.h"
#include<iostream>
#include"Math.h"
#include"Elevator.h"

Drivetrain::Drivetrain(uint8_t L1, uint8_t L2, uint8_t R1, uint8_t R2, uint8_t gyro, int PCM, int shifterFWD, int shifterREV, Elevator* ele){

	list = new struct driveList;

	list->gyro = new PigeonIMU(gyro);

	list->gyro->SetFusedHeading(0, 10);

	list->prefs = Preferences::GetInstance();

	//list->accel = new BuiltInAccelerometer(Accelerometer::kRange_8G);

	list->Left1 = new WPI_TalonSRX(L1);
	list->Left2 = new WPI_TalonSRX(L2);
	list->Right1 = new WPI_TalonSRX(R1);
	list->Right2 = new WPI_TalonSRX(R2);

	list->L2ID = L2;
	list->R2ID = R2;

	list->shifter = new DoubleSolenoid(PCM, shifterFWD, shifterREV);

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

	list->Left2->ConfigNominalOutputForward(0, 10);
	list->Right2->ConfigNominalOutputForward(0, 10);
	list->Left2->ConfigNominalOutputReverse(0, 10);
	list->Right2->ConfigNominalOutputReverse(0, 10);

	list->prefs->PutDouble("P", list->P);
	list->prefs->PutDouble("I", list->I);
	list->prefs->PutDouble("D", list->D);
	list->prefs->PutDouble("F", list->F);

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

	list->shifter->Set(frc::DoubleSolenoid::Value::kForward);

	std::cout << "Drivetrain Done" << std::endl;

	list->currTime = list->accelTimeLo;
	list->lastTime = list->currTime;

}

void Drivetrain::Drive(float fwd, float trn, bool autoHeading){

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

void Drivetrain::Shift(){

	if(list->shifter->Get() == frc::DoubleSolenoid::Value::kForward){
		list->shifter->Set(frc::DoubleSolenoid::Value::kReverse);
		list->HiGear = true;
		list->Left2->ConfigClosedloopRamp(list->accelTimeHi, 10);
		list->Right2->ConfigClosedloopRamp(list->accelTimeHi, 10);
	}else{
		list->shifter->Set(frc::DoubleSolenoid::Value::kForward);
		list->HiGear = false;
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

	frc::Wait(0.1);

	list->Left2->SetSelectedSensorPosition(left, 0, 10);
	list->Right2->SetSelectedSensorPosition(right, 0, 10);

}

double Drivetrain::getGyroAngle(){
	return list->gyro->GetFusedHeading();
}

void Drivetrain::setGyroAngle(double angle){
	list->gyro->SetFusedHeading(angle, 10);
}

float * Drivetrain::GetCurr(){

	list->current[0] = list->Left2->GetOutputCurrent();
	list->current[1] = list->Right2->GetOutputCurrent();

	return list->current;
}
