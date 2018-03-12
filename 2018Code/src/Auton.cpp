/*
 * Auton.cpp
 *
 *  Created on: Jan 16, 2018
 *      Author: feds201
 */
#include"Auton.h"
#include<iostream>



Auton::Auton(Drivetrain * drive){

	list = new struct AutonList;

	list->drive = drive;

	std::cout << "Auton Init" << std::endl;

}

void Auton::Drive(double speed, double dist, float current){

if(list->hasTurned)
	dist += list->offSet;

  dist *= list->encCountsPerRev*(list->gearRatioLo*(1/(2*list->pi*list->wheelR)));

list->drive->SetEncPos(0, 0);
list->drive->setGyroAngle(0);

while((abs(list->drive->GetEncPos()[1])+abs(list->drive->GetEncPos()[0]))/2 < dist && (list->drive->GetCurr()[0]+list->drive->GetCurr()[1])/2 < current){
	list->drive->Drive(speed, 0, true);
}
	list->drive->Drive(0, 0, true);

}
void Auton::Rotate(double angle){

	list->hasTurned = true;

	list->drive->setGyroAngle(0);

	if(angle < 0){

		while(list->drive->getGyroAngle() > angle){
			list->drive->Drive(0, 0.5, false);
		}

	}else{

		while(list->drive->getGyroAngle() < angle){
			list->drive->Drive(0, -0.5, false);
		}

	}

	list->drive->Drive(0, 0, false);
	list->drive->setGyroAngle(0);
	list->drive->SetEncPos(0, 0);
	frc::Wait(1);
	list->drive->setGyroAngle(0);
	list->drive->SetEncPos(0, 0);


}

void Auton::Arc(double speed, double turnAngle, double radius){

list->timer.Reset();
list->timer.Start();
list->drive->setGyroAngle(0);

list->Vc = speed;
list->Rc = radius;

list->Wc = list->Vc/list->Rc;

while(list->drive->getGyroAngle() < turnAngle){

	list->t = list->timer.Get();

	list->thetac = list->Wc*list->t;
	list->theta = list->drive->getGyroAngle();
	list->thetaErr = list->theta-list->thetac;

	list->Vi += list->thetaErr*list->Ptheta;
	list->Vo -= list->thetaErr*list->Ptheta;

	list->Si = list->drive->GetEncVel()[1];
	list->So = list->drive->GetEncVel()[0];

	list->Si = (((list->Si*10)/list->gearRatioLo)/list->encCountsPerRev)*((2*list->pi)*list->wheelR);
	list->So = (((list->So*10)/list->gearRatioLo)/list->encCountsPerRev)*((2*list->pi)*list->wheelR);

	list->Ra = (list->Track/2)*(((list->Si/list->So)+1)/(1-(list->Si/list->So)));
	list->Va = list->Si/(1 - (0.5*(list->Track/list->Ra)));

	list->VErr = list->Va-list->Vc;
	list->Vi -= list->VErr*list->Pv;
	list->Vo -= list->VErr*list->Pv;

	list->VoSetPt = (((list->Si/10)*list->gearRatioLo)*list->encCountsPerRev)/((2/list->pi)/list->wheelR);
	list->ViSetPt = (((list->So/10)*list->gearRatioLo)*list->encCountsPerRev)/((2/list->pi)/list->wheelR);

	list->drive->Set(list->VoSetPt, list->ViSetPt);


}





}

