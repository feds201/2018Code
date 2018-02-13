/*
 * Elevator.cpp
 *
 *  Created on: Jan 24, 2018
 *      Author: feds201
 */


#include"Elevator.h"
#include<iostream>

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

	//Main point is to tell motor what type of motor it has. FeedbackDevice- type of motor, pidIDX(0)- Don't know, timeout in ms(10)- Don't know
	list->motor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);

	//Sets hiPres DoubleSolenoid Object to Enum type kReverse. Makes one side of solenoid on.
	list->hiPresToggle->Set(frc::DoubleSolenoid::Value::kReverse);
	list->solenoid->Set(frc::DoubleSolenoid::Value::kReverse);

	//Prints to driver station
	std::cout << "Ele Init" << std::endl;

}

//Moves elevator, still need to figure out what negative and positive speeds mean
void Elevator::Move(double speed){

	//Ramping vv
	if(speed != 0 && speed < 0 && list->motor->GetSelectedSensorPosition(0) < 2000){
		speed = -0.2;
	}else if(speed != 0 && speed > 0 && list->motor->GetSelectedSensorPosition(0) > 10000){
		speed = 0.2;
	}

	//If speed < 0 and elevator is at bottom, set the motor to the speed
	if(speed < 0 && list->bottomlimit->Get()){
		list->motor->Set(speed);
	}else if(speed > 0 && list->toplimit->Get()){ //Else the speed > 0 and the elevator is at the top
		list->motor->Set(speed);
	}else{
		list->motor->Set(0); //Else motor is 0
	}

}

//Still questions on this vv

//Eject cube to high pressure
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

//Eject cube to low pressure
void Elevator::PushLo(){

	//If cube pushing solenoid is in kForward, switch to low pressure, and retract cube pusher
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



}

