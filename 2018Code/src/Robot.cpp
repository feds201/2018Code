#include"WPILib.h"
#include"ctre/Phoenix.h"
#include"Drivetrain.h"
#include"Auton.h"
#include"EdgeDetection.h"
#include "Elevator.h"
#include "Pickup.h"
#include <SmartDashboard/SendableChooser.h>
#include <iostream>
#include"CameraServer.h"

class Robot : public frc::SampleRobot {

	Drivetrain drive;
	Joystick joy, joy2;
	Compressor comp;
	Auton auton;
	Edge shift, pickup, eject, ejectHi;
	Edge clamp;
	Pickup pick;
	Elevator ele;
	frc::SendableChooser<std::string> chooser;
	frc::SendableChooser<std::string> lrchooser;
	DoubleSolenoid climber;
	Edge climb;
	cs::UsbCamera cam1;
	cs::UsbCamera cam2;
	PowerDistributionPanel pdp;
	AnalogInput pressure;



public:
	Robot():
	drive(1, 2, 4, 3, 0, 8, 1, 2, &ele),
	joy(0),
	joy2(1),
	comp(8),
	auton(&drive, this),
	shift(joy.GetRawButton(1)),
	pickup(joy2.GetRawButton(5)),
	eject(joy2.GetRawButton(1)),
	ejectHi(joy2.GetRawButton(4)),
	clamp(joy2.GetRawButton(6)),
	pick(0, 7, 8, 3, 4, 5, 6),
	ele(5, 8, 6, 7, 3, 4, 5, 0, 1, this),
	climber(0, 1, 2),
	cam1(),
	cam2(),
	pdp(0),
	pressure(0)

	{


	}

	void RobotInit() {

		drive.SetEncPos(0, 0);
		drive.setGyroAngle(0);
		drive.SetEncPos(0, 0);
		drive.setGyroAngle(0);

		const std::string sw = "Switch";
		const std::string sc = "Scale";
		const std::string l = "Left";
		const std::string r = "Right";
		const std::string m = "Middle";
		const std::string d = "Default";

		chooser.AddDefault("Switch", sw);
		chooser.AddObject("Scale", sc);
		chooser.AddObject("Default", d);

		lrchooser.AddDefault("Left", l);
		lrchooser.AddObject("Right", r);
		lrchooser.AddObject("Middle", m);

		SmartDashboard::PutData("Auto Modes", &chooser);
		SmartDashboard::PutData("Select Side", &lrchooser);
		SmartDashboard::PutString("Game Data", frc::DriverStation::GetInstance().GetGameSpecificMessage());

		cam1 = CameraServer::GetInstance()->StartAutomaticCapture("Cam1 ", 0);
		cam2 = CameraServer::GetInstance()->StartAutomaticCapture("Cam2", 1);

	}


	void Autonomous() {

		drive.SetEncPos(0, 0);
		drive.setGyroAngle(0);

		double swDist = 129; //-3 in for bumpers //-25" total
		double alleyDist = 172; //-3 in for bumpers //-56" total
		double scDist = 295; //-3 in for bumpers //-15" total
		double swAlley = 103; //
		double scAlley = 136;
		double swDistFinal = 40;
		double scDistFinal = 13;
		double swApproachDist = 40; //-3 in for bumpers //Added 12 in bacause robot wasn't contacting switch
		double scApproachDist = 8.4; //-3 in for bumpers
		double SpeedFast = -.7; // -0.7
		double SpeedSlow = -.4; // -0.4
		double SpeedZoomi = -0.9;
		int eleMiddle = 17600;
		int eleHigh = 33000;

		/*
		 *
		 *
		 * COMMENTED OUT FINAL APPROACH ON ALL CROSS FEILD SCALE AUTONS
		 *
		 *
		 *
		 */

		std::string selected = chooser.GetSelected();
		std::string side = lrchooser.GetSelected();
		std::string gameInfo = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		DriverStation::Alliance alliance = DriverStation::GetInstance().GetAlliance();
		std::string swSide = "";
		std::string scSide = "";

		if(gameInfo[0] == 'L')
			swSide = "Left";
		else
			swSide = "Right";

		if(gameInfo[1] == 'L')
			scSide = "Left";
		else
			scSide = "Right";

		std::cout << "Auto Selected " << selected << std::endl;
		std::cout << "Side Selected " << side << std::endl;
		std::cout << "Switch Side " << swSide << std::endl;
		std::cout << "Scale Side " << scSide << std::endl;

		drive.SetEncPos(0, 0);
		drive.setGyroAngle(0);


		int sel = 0;

				if (side == "Left"){
					if(selected == "Switch"){
						if(swSide == "Left"){
							sel = 0;
							//Robot On Left, Switch on Left
						}else{
							sel = 1;
							//Robot On Left, Switch on Right
						}
					}else if(selected == "Scale"){
						if(scSide == "Left"){
							sel = 2;
							//Robot On Left, Scale on Left
						}else{
							sel = 3;
							//Robot On Left, Scale on Right
						}
					}else{
						sel = 201;
					}


				}else if(side == "Right"){
					if(selected == "Switch"){
						if(swSide == "Left"){
							sel = 4;
							//Robot On Right, Switch on Left
						}else{
							sel = 5;
							//Robot On Right, Switch On Right
						}
					}else if(selected == "Scale"){
						if(scSide == "Left"){
							sel = 6;
							//Robot On Right, Scale on Left
						}else{
							sel = 7;
							//Robot On Right, Scale On Right
					}
				}else{
					sel = 201;
				}
				}else{
					if(selected != "Default"){
						if(swSide == "Left"){
							//Robot In Middle Switch On Left

							sel = 9;

						}else{
							//Robot In Middle Switch On Right

							sel = 8;

						}

					}else{
						sel = 201;
					}
				}

				drive.SetEncPos(0, 0);
				drive.setGyroAngle(0);

				switch(sel){
				case 0 :
					//LSL
					SmartDashboard::PutString("Auton Info", "Robot on left, going for switch on left");

						if(alliance == DriverStation::Alliance::kBlue){
							auton.Drive(SpeedFast, swDist, 100);
							auton.Rotate(-52);
							auton.Drive(SpeedSlow, swApproachDist, 9);
							drive.Drive(0, 0, false);
							pick.Grab();
							frc::Wait(1);
							ele.TargetHeight(eleMiddle);
							ele.PushLo(true);
						}else if(alliance == DriverStation::Alliance::kRed){
							auton.Drive(SpeedFast, swDist, 100);
							auton.Rotate(-52);
							auton.Drive(SpeedSlow, swApproachDist, 9);
							drive.Drive(0, 0, false);
							pick.Grab();
							frc::Wait(1);
							ele.TargetHeight(eleMiddle);
							ele.PushLo(true);
						}
				break;

				case 1 :
					//LSR
					SmartDashboard::PutString("Auton Info", "Robot on left, going for switch on right");

					if(alliance == DriverStation::Alliance::kBlue){
						auton.Drive(SpeedZoomi, alleyDist, 100);
						auton.Rotate(-52);
						auton.Drive(SpeedZoomi, swAlley+25, 100);
						auton.Rotate(-52);
						auton.Drive(SpeedSlow, swDistFinal, 8);
						drive.Drive(0, 0, false);
						pick.Grab();
						frc::Wait(1);
						ele.TargetHeight(eleMiddle+7700);
						ele.PushLo(true);
					}else if(alliance == DriverStation::Alliance::kRed){
						auton.Drive(SpeedZoomi, alleyDist, 100);
						auton.Rotate(-52);
						auton.Drive(SpeedZoomi, swAlley+25, 100);
						auton.Rotate(-52);
						auton.Drive(SpeedSlow, swDistFinal, 8);
						drive.Drive(0, 0, false);
						pick.Grab();
						frc::Wait(1);
						ele.TargetHeight(eleMiddle+7700);
						ele.PushLo(true);
					}
				break;

				case 2 :
					//LSCL
					SmartDashboard::PutString("Auton Info", "Robot on left, going for scale on left");

					if(alliance == DriverStation::Alliance::kBlue){
						auton.Drive(SpeedFast, scDist, 100);
						auton.Rotate(-52);
						//auton.Drive(SpeedSlow, 5, 100);
						drive.Drive(0, 0, false);
						pick.Grab();
						frc::Wait(1);
						ele.TargetHeight(eleHigh);
						ele.PushLo(true);
					}else if(alliance == DriverStation::Alliance::kRed){

						auton.Drive(SpeedFast, scDist, 100);
						auton.Rotate(-52);
						//auton.Drive(SpeedSlow, 5, 100);
						drive.Drive(0, 0, false);
						pick.Grab();
						frc::Wait(1);
						ele.TargetHeight(eleHigh);
						ele.PushLo(true);

					}
				break;

				case 3 :
					//LSCR
					SmartDashboard::PutString("Auton Info", "Robot on left, going for scale on right");

						if(alliance == DriverStation::Alliance::kBlue){

						auton.Drive(SpeedZoomi, alleyDist, 100);
						auton.Rotate(-52);
						auton.Drive(SpeedZoomi, scAlley+25, 100);
						auton.Rotate(52);
						auton.Drive(SpeedSlow, scDistFinal+12, 100);
						drive.Drive(0, 0, false);
						pick.Grab();
						frc::Wait(1);
						ele.TargetHeight(eleHigh);
						ele.PushLo(true);

						}else if(alliance == DriverStation::Alliance::kRed){

							auton.Drive(SpeedZoomi, alleyDist, 100);
							auton.Rotate(-52);
							auton.Drive(SpeedZoomi, scAlley, 100);
							auton.Rotate(52);
							auton.Drive(SpeedSlow, scDistFinal+12, 100);
							drive.Drive(0, 0, false);
							pick.Grab();
							frc::Wait(1);
							ele.TargetHeight(eleHigh);
							ele.PushLo(true);


						}
				break;

				case 4 :
					//RSL

					if(alliance == DriverStation::Alliance::kBlue){

					SmartDashboard::PutString("Auton Info", "Robot on right, going for switch on left");
					auton.Drive(SpeedZoomi, alleyDist, 100);
					auton.Rotate(52); //Subtracted 3 deg
					auton.Drive(SpeedZoomi, (swAlley+25), 100);
					auton.Rotate(52);
					auton.Drive(SpeedSlow, swDistFinal, 8);
					drive.Drive(0, 0, false);
					pick.Grab();
					frc::Wait(1);
					ele.TargetHeight(eleMiddle+7700);
					ele.PushLo(true);

					}else if(alliance == DriverStation::Alliance::kRed){

						SmartDashboard::PutString("Auton Info", "Robot on right, going for switch on left");
						auton.Drive(SpeedZoomi, alleyDist, 100);
						auton.Rotate(52); //Subtracted 3 deg
						auton.Drive(SpeedZoomi, (swAlley+25), 100);
						auton.Rotate(52);
						auton.Drive(SpeedSlow, swDistFinal, 8);
						drive.Drive(0, 0, false);
						pick.Grab();
						frc::Wait(1);
						ele.TargetHeight(eleMiddle+7700);
						ele.PushLo(true);


					}
				break;

				case 5 :
					//RSR

					if(alliance == DriverStation::Alliance::kBlue){

					SmartDashboard::PutString("Auton Info", "Robot on right, going for switch on right");
					auton.Drive(SpeedFast, swDist, 100);
					auton.Rotate(52); //Subtracted 3 deg
					auton.Drive(SpeedSlow, swApproachDist, 9);
					drive.Drive(0, 0, false);
					pick.Grab();
					frc::Wait(1);
					ele.TargetHeight(eleMiddle);
					ele.PushLo(true);
					}else if(alliance == DriverStation::Alliance::kRed){

						SmartDashboard::PutString("Auton Info", "Robot on right, going for switch on right");
						auton.Drive(SpeedFast, swDist, 100);
						auton.Rotate(52); //Subtracted 3 deg
						auton.Drive(SpeedSlow, swApproachDist, 9);
						drive.Drive(0, 0, false);
						pick.Grab();
						frc::Wait(1);
						ele.TargetHeight(eleMiddle);
						ele.PushLo(true);


					}
				break;

				case 6 :
					//RSCL

					if(alliance == DriverStation::Alliance::kBlue){

					SmartDashboard::PutString("Auton Info", "Robot on right, going for scale on left");
					auton.Drive(SpeedZoomi, alleyDist, 100);
					auton.Rotate(52); //Subtracted 3 deg
					auton.Drive(SpeedZoomi, (scAlley+25), 100);
					auton.Rotate(-52);
					auton.Drive(SpeedSlow, scDistFinal, 100);
					drive.Drive(0, 0, false);
					pick.Grab();
					frc::Wait(1);
					ele.TargetHeight(eleHigh);
					ele.PushLo(true);
					}else if(alliance == DriverStation::Alliance::kRed){

						SmartDashboard::PutString("Auton Info", "Robot on right, going for scale on left");
						auton.Drive(SpeedZoomi, alleyDist, 100);
						auton.Rotate(52); //Subtracted 3 deg
						auton.Drive(SpeedZoomi, (scAlley+25), 100);
						auton.Rotate(-52);
						auton.Drive(SpeedSlow, scDistFinal, 100);
						drive.Drive(0, 0, false);
						pick.Grab();
						frc::Wait(1);
						ele.TargetHeight(eleHigh);
						ele.PushLo(true);


					}
				break;

				case 7 :
					//RSCR

					if(alliance == DriverStation::Alliance::kBlue){

					SmartDashboard::PutString("Auton Info", "Robot on right, going for scale on right");
					auton.Drive(SpeedFast, scDist, 100);
					auton.Rotate(52); //Subtracted 3 deg
					//auton.Drive(SpeedSlow, 5, 100);
					drive.Drive(0, 0, false);
					pick.Grab();
					frc::Wait(1);
					ele.TargetHeight(eleHigh);
					ele.PushLo(true);
					}else if(alliance == DriverStation::Alliance::kRed){

						SmartDashboard::PutString("Auton Info", "Robot on right, going for scale on right");
						auton.Drive(SpeedFast, scDist, 100);
						auton.Rotate(52); //Subtracted 3 deg
						//auton.Drive(SpeedSlow, 5, 100);
						drive.Drive(0, 0, false);
						pick.Grab();
						frc::Wait(1);
						ele.TargetHeight(eleHigh);
						ele.PushLo(true);


					}

				break;

				case 8 :

					SmartDashboard::PutString("Auton Info", "Smart Switch, right");

					if(alliance == DriverStation::Alliance::kBlue){

						auton.Drive(SpeedFast, 20, 100);
						auton.Rotate(-52);
						auton.Drive(SpeedFast, 24, 100);
						auton.Rotate(52);
						drive.SetEncPos(0, 0);
						auton.Drive(SpeedSlow, 83, 9);
						drive.Drive(0, 0, false);
						pick.Grab();
						frc::Wait(1);
						ele.TargetHeight(eleMiddle);
						ele.PushLo(true);

					}else if(alliance == DriverStation::Alliance::kRed){

						auton.Drive(SpeedFast, 20, 100);
						auton.Rotate(-52);
						auton.Drive(SpeedFast, 24, 100);
						auton.Rotate(52);
						drive.SetEncPos(0, 0);
						auton.Drive(SpeedSlow, 83, 9);
						drive.Drive(0, 0, false);
						pick.Grab();
						frc::Wait(1);
						ele.TargetHeight(eleMiddle);
						ele.PushLo(true);


					}



				break;

				case 9 :

					SmartDashboard::PutString("Auton Info", "Smart Switch, left");

					if(alliance == DriverStation::Alliance::kBlue){

						auton.Drive(SpeedFast, 20, 100);
						auton.Rotate(52);
						auton.Drive(SpeedFast, 26, 100);
						auton.Rotate(-52);
						drive.SetEncPos(0, 0);
						auton.Drive(SpeedSlow, 83, 9);
						drive.Drive(0, 0, false);
						pick.Grab();
						frc::Wait(1);
						ele.TargetHeight(eleMiddle);
						ele.PushLo(true);

					}else if(alliance == DriverStation::Alliance::kRed){

						auton.Drive(SpeedFast, 20, 100);
						auton.Rotate(52);
						auton.Drive(SpeedFast, 26, 100);
						auton.Rotate(-52);
						drive.SetEncPos(0, 0);
						auton.Drive(SpeedSlow, 83, 9);
						drive.Drive(0, 0, false);
						pick.Grab();
						frc::Wait(1);
						ele.TargetHeight(eleMiddle);
						ele.PushLo(true);

					}



				break;

				default :
					//Just Go FWD
					SmartDashboard::PutString("Auton Info", "No Auton, Just going fwd and stoping");
						auton.Drive(SpeedFast, 30, 12
								);
				break;

				}



		/*
		drive.SetEncPos(0, 0);
		drive.setGyroAngle(0);
		auton.Drive(-0.4, 152);
		auton.Rotate(-90);
		drive.SetEncPos(0, 0);
		drive.setGyroAngle(0);
		drive.SetEncPos(0, 0);
		auton.Drive(-0.3, 18);
		m1.Set(1);
		m2.Set(1);
		frc::Wait(4);s
		m1.Set(0);
		m2.Set(0);
		drive.setGyroAngle(0);
		auton.Rotate(-90);
		drive.Set(0, 0);
		drive.setGyroAngle(0);
		auton.Drive(-0.4, 152);
*/
		/*
		drive.setGyroAngle(0);
		drive.SetEncPos(0, 0);
		auton.Drive(-0.4, 298);
		drive.setGyroAngle(0);
		auton.Rotate(-90);
		drive.SetEncPos(0, 0);
		drive.setGyroAngle(0);
		auton.Drive(-0.3, 11);
		m1.Set(1);
		m2.Set(1);
		frc::Wait(4);
		m1.Set(0);
		m2.Set(0);

*/


	}

	float deadzone(float f){

			if (fabs(f) < .15)
				return 0.0f;
			else{
				if(f > 0)
					return(f-.15)/(1-.15);
				else
					return(f+.15)/(1-.15);
			}


		}


	void OperatorControl() override {

		comp.SetClosedLoopControl(true);
		drive.SetEncPos(0, 0);
		drive.setGyroAngle(0);
		climber.Set(frc::DoubleSolenoid::Value::kReverse);
		bool panik = false;
		drive.SetEncPos(0, 0);
		int press;

		while (IsOperatorControl() && IsEnabled()) {

			if (joy2.GetRawButton(8))
				panik = true;
			else if(joy2.GetRawButton(9))
				panik = false;

			//if(joy.GetRawButton(8)){
				//eleCam.SetFPS(15);
				//climbCam.SetFPS(0);
			//}else if(joy.GetRawButton(7)){
				//eleCam.SetFPS(0);
				//climbCam.SetFPS(15);
			//}
			pickup.update(joy2.GetRawButton(6));
			eject.update(joy2.GetRawButton(1));
			ejectHi.update(joy2.GetRawButton(4));
			clamp.update(joy2.GetRawButton(5));
			//bottom.update(joy2.GetRawButton(1));
			//middle.update(joy2.GetRawButton(2));
			//top.update(joy2.GetRawButton(4));

			shift.update(joy.GetRawButton(1));


			if(pickup.isPressed())
				pick.Toggle();

				ele.PushLo(eject.getState());

			if(ejectHi.isPressed())
				ele.PushHi();

			if(clamp.isPressed()){
				pick.Grab();
				std::cout << "Clamping" << std::endl;
			}
			//if(bottom.isPressed())
				//ele.Bottom();

			//if(middle.isPressed())
				//ele.Middle();

			//if(top.isPressed())
				//ele.Top();

			if(shift.isPressed())
				drive.Shift();

			pick.WheelSpeed(deadzone(joy2.GetRawAxis(4))); //:(

			if(!panik)
			ele.Move(deadzone(-joy2.GetRawAxis(1)));

			drive.Drive(deadzone(joy.GetRawAxis(1)), deadzone(joy.GetRawAxis(4)), false);

			//ele.Refresh();

			if(joy2.GetRawButton(7))
				climber.Set(frc::DoubleSolenoid::Value::kForward);
			else if(joy2.GetRawButton(8))
				climber.Set(frc::DoubleSolenoid::Value::kReverse);

			SmartDashboard::PutNumber("Left Encoder Vel", drive.GetEncVel()[0]);
			SmartDashboard::PutNumber("Right Encoder Vel", drive.GetEncVel()[1]);
			//SmartDashboard::PutNumber("Total Amps", pdp.GetTotalCurrent());
			//SmartDashboard::PutNumber("Total Power", pdp.GetTotalPower());

			press = (double)((((double)pressure.GetValue()-404.0)/3418.0)*120.0);

			SmartDashboard::PutBoolean("Good To Climb?", press >= 45 ? true : false);

			SmartDashboard::PutNumber("Pressure", press); //(((pressure.GetValue()-404)/3418)*120))

			//Logger::Data

			//Logger::instance()->logCSV();



			frc::Wait(0.005);
		}
	}
	void Test() override {}

};

START_ROBOT_CLASS(Robot)
