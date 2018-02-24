/*
 * Elevator.cpp
 *
 *  Created on: Jan 24, 2018
 *      Author: feds201
 */


#include"Elevator.h"
#include<iostream>
#include"WPILib.h"
#include"ctre/Phoenix.h"

Elevator::Elevator(uint8_t motorID, int PCM, int fwdsolenoid, int revsolenoid, int presToggleHi, int presToggleHiOff, int presToggleLo,  int tlimit, int blimit){

	//List points to a new copy of Elist
	list = new struct EList;

	//New SRX object with motorID
	list->motor = new WPI_TalonSRX(motorID);

	//New DoubleSolenoid Object PCM- ID of PCM, fwd- Port on PCM that solenoid is hooked up to, rev- Second port on PCM
	list->solenoid = new DoubleSolenoid(PCM, fwdsolenoid, revsolenoid);

	//ToggleHi- port activated on PCM when High Pressure is wanted, ToggleHiOff is opposite
	list->hiPresToggle = new DoubleSolenoid(PCM, presToggleHi, presToggleHiOff);

	//New Solenoid Object, Only one port (ToggleLo) because it is a single solenoid
	list->loPresToggle = new Solenoid(PCM, presToggleLo);

	//New DI Object, tlimit- Port on DIO (reads port to see if switch is true or false)
	list->toplimit = new DigitalInput(tlimit);
	list->bottomlimit = new DigitalInput(blimit);

	//Main point is to tell motor what type of encoder it has. FeedbackDevice- type of encoder, pidIDX(0)- Don't know, timeout in ms(10)- Don't know
	list->motor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);

	//Sets hiPres DoubleSolenoid Object to Enum type kReverse. Makes one side of solenoid on.
	list->hiPresToggle->Set(frc::DoubleSolenoid::Value::kReverse);
	list->solenoid->Set(frc::DoubleSolenoid::Value::kForward);

	//Prints to console in driver station, not shuffleboard
	std::cout << "Ele Init" << std::endl;

	//list->motor->Config_kP(0, 0, 10);


	list->motor->SetSelectedSensorPosition(0, 0, 20);

}

//Moves elevator, still need to figure out what negative and positive speeds mean
void Elevator::Move(double speed){

	/*
		if(speed != 0 && speed < 0 && list->motor->GetSelectedSensorPosition(0) < 2000){
			speed = -0.2;
		}else if(speed != 0 && speed > 0 && list->motor->GetSelectedSensorPosition(0) > 10000){
			speed = 0.2;
		}
	*/
		//if(speed == 0){

			//list->motor->Set(ControlMode::Position, list->pos);

		//}else{

		//Get the height of the elevator in encoder countd and put it on the shuffleboard
		list->pos = list->motor->GetSelectedSensorPosition(0);
		SmartDashboard::PutNumber("Ele Enc", list->pos);

		//Percent output is -1 to 1 enumerator

		//Moves ele with restrictions on moving up and down
		if(speed < 0 && list->bottomlimit->Get()){
			list->motor->Set(ControlMode::PercentOutput, speed);
		}else if(speed > 0 && list->toplimit->Get()){
			list->motor->Set(ControlMode::PercentOutput, speed);
		}else{
			list->motor->Set(ControlMode::PercentOutput, 0.07);
		}
	//}
}

//Moves elevator to target height
void Elevator::TargetHeight(double enc){

	//While the current height is less than the target height keep moving it (what does it do when its greater)
	while(list->motor->GetSelectedSensorPosition(0) < enc){

		Move(0.8);
	}

	//Stop elevator
	Move(0);

}

//Don't use
void Elevator::PushHi(){

	//If cube pushing solenoid is in kForward, switch low pressure solenoid to true, and retract cube pusher
	if(list->solenoid->Get() == frc::DoubleSolenoid::Value::kForward){
		list->hiPresToggle->Set(frc::DoubleSolenoid::Value::kReverse);
		list->loPresToggle->Set(true);
		list->solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
	}
	//If not, set high pressure solenoid to true, and push piston forward
	else{
		list->hiPresToggle->Set(frc::DoubleSolenoid::Value::kForward);
		list->loPresToggle->Set(false);
		list->solenoid->Set(frc::DoubleSolenoid::Value::kForward);
	}


}


void Elevator::PushLo(bool push){

	if(list->solenoid->Get() == frc::DoubleSolenoid::Value::kForward){
		list->hiPresToggle->Set(frc::DoubleSolenoid::Value::kReverse);
		list->loPresToggle->Set(true);
		list->solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
	}
	//If not, set to low pressure, push piston forward
	else{
		list->hiPresToggle->Set(frc::DoubleSolenoid::Value::kReverse);
		list->loPresToggle->Set(true);
		list->solenoid->Set(frc::DoubleSolenoid::Value::kForward);
	}

	if(push){
		list->solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
	}else{
		list->solenoid->Set(frc::DoubleSolenoid::Value::kForward);
	}


}

//Gets the height of the elevator in encoder counts
double  Elevator::getHeight(){
	return list->motor->GetSelectedSensorPosition(0);
}
