#include <Drivetrain.h>
#include"WPILib.h"
#include"ctre/Phoenix.h"
#include<iostream>
#include"Math.h"

Drivetrain::Drivetrain(uint8_t L1, uint8_t L2, uint8_t R1, uint8_t R2, uint8_t gyro, int PCM, int shifterFWD, int shifterREV){

	list = new struct driveList;

	list->gyro = new PigeonIMU(gyro);

	list->accelValues = new int16_t[3];

	list->gyro->SetFusedHeading(0, 10);

	list->prefs = Preferences::GetInstance();

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

}

void Drivetrain::Drive(float fwd, float trn, bool autoHeading){

	SmartDashboard::PutNumber("Gyro", getGyroAngle());
	SmartDashboard::PutNumber("LEnc", GetEncPos()[0]);
	SmartDashboard::PutNumber("REnc", GetEncPos()[1]);

	list->gyro->GetBiasedAccelerometer(list->accelValues);

	list->accelX = list->accelValues[0]/16384;
	list->accelY = list->accelValues[1]/16384;
	list->accelZ = list->accelValues[2]/16384;

	fwd *= 1-pow((list->accelX/list->MAXAccel), 3);

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

	if(list->shifter->Get() == frc::DoubleSolenoid::Value::kForward)
		list->shifter->Set(frc::DoubleSolenoid::Value::kReverse);
	else
		list->shifter->Set(frc::DoubleSolenoid::Value::kForward);

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

double Drivetrain::getGyroAngle(){
	return list->gyro->GetFusedHeading();
}

void Drivetrain::setGyroAngle(double angle){
	list->gyro->SetFusedHeading(angle, 10);
}
