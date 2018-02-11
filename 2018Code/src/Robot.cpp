
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
	cs::UsbCamera eleCam;
	cs::UsbCamera climbCam;


public:
	Robot():

	drive(1, 2, 4, 3, 0, 8, 1, 2),
	joy(0),
	joy2(1),
	comp(8),
	auton(&drive),
	shift(joy.GetRawButton(1)),
	pickup(joy2.GetRawButton(5)),
	eject(joy2.GetRawButton(1)),
	ejectHi(joy2.GetRawButton(4)),
	clamp(joy2.GetRawButton(6)),
	pick(0, 7, 8, 3, 4, 5, 6),
	ele(5, 8, 6, 7, 3, 4, 5, 0, 1),
	climber(0, 1, 2),
	eleCam(),
	climbCam()

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

		chooser.AddDefault("Switch", sw);
		chooser.AddObject("Scale", sc);

		lrchooser.AddDefault("Left", l);
		lrchooser.AddObject("Right", r);

		SmartDashboard::PutData("Auto Modes", &chooser);
		SmartDashboard::PutData("Select Side", &lrchooser);
		SmartDashboard::PutString("Game Data", frc::DriverStation::GetInstance().GetGameSpecificMessage());

		eleCam = CameraServer::GetInstance()->StartAutomaticCapture();
		climbCam = CameraServer::GetInstance()->StartAutomaticCapture();
		eleCam.SetExposureAuto();
		eleCam.SetResolution(480, 360);
		eleCam.SetFPS(15);
		climbCam.SetExposureAuto();
		climbCam.SetResolution(480, 360);
		climbCam.SetFPS(0);

	}

	void Autonomous() {



		double swDist = 154;
		double alleyDist = 222;
		double scDist = 310;
		double swAlley = 174.5;
		double scAlley = 188.2;
		double swDistFinal = 25.75;
		double scDistFinal = 50;
		double swApproachDist = 25;
		double scApproachDist = 11.4;
		double SpeedFast = -.5;
		double SpeedSlow = -.3;

		std::string selected = chooser.GetSelected();
		std::string side = lrchooser.GetSelected();
		std::string gameInfo = frc::DriverStation::GetInstance().GetGameSpecificMessage();
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
					}else{
						if(scSide == "Left"){
							sel = 2;
							//Robot On Left, Scale on Left
						}else{
							sel = 3;
							//Robot On Left, Scale on Right
						}
					}


				}else{
					if(selected == "Switch"){
						if(swSide == "Left"){
							sel = 4;
							//Robot On Right, Switch on Left
						}else{
							sel = 5;
							//Robot On Right, Switch On Right
						}
					}else{
						if(scSide == "Left"){
							sel = 6;
							//Robot On Right, Scale on Left
						}else{
							sel = 7;
							//Robot On Right, Scale On Right
					}
				}
				}


				switch(sel){
				case 0 :
					//LSL
					SmartDashboard::PutString("Auton Info", "Robot on left, going for switch on left");
						auton.Drive(SpeedFast, swDist);
						auton.Rotate(-90);
						auton.Drive(SpeedSlow, swApproachDist);
						ele.PushLo();

				break;

				case 1 :
					//LSR
					SmartDashboard::PutString("Auton Info", "Robot on left, going for switch on right");
						auton.Drive(SpeedFast, alleyDist);
						auton.Rotate(-90);
						auton.Drive(SpeedFast, swAlley);
						auton.Rotate(-90);
						auton.Drive(SpeedSlow, swDistFinal);
						ele.PushLo();

				break;

				case 2 :
					//LSCL
					SmartDashboard::PutString("Auton Info", "Robot on left, going for scale on left");
						auton.Drive(SpeedFast, scDist);
						auton.Rotate(-90);
						auton.Drive(SpeedSlow, scApproachDist);
						ele.PushLo();

				break;

				case 3 :
					//LSCR
					SmartDashboard::PutString("Auton Info", "Robot on left, going for scale on right");
						auton.Drive(SpeedFast, alleyDist);
						auton.Rotate(-90);
						auton.Drive(SpeedFast, scAlley);
						auton.Rotate(90);
						auton.Drive(SpeedSlow, scDistFinal);
						ele.PushLo();

				break;

				case 4 :
					//RSL
					SmartDashboard::PutString("Auton Info", "Robot on right, going for switch on left");
					auton.Drive(SpeedFast, alleyDist);
					auton.Rotate(90);
					auton.Drive(SpeedFast, swAlley);
					auton.Rotate(90);
					auton.Drive(SpeedSlow, swDistFinal);
					ele.PushLo();

				break;

				case 5 :
					//RSR
					SmartDashboard::PutString("Auton Info", "Robot on right, going for switch on right");
					auton.Drive(SpeedFast, swDist);
					auton.Rotate(90);
					auton.Drive(SpeedSlow, swApproachDist);
					ele.PushLo();

				break;

				case 6 :
					//RSCL
					SmartDashboard::PutString("Auton Info", "Robot on right, going for scale on left");
					auton.Drive(SpeedFast, alleyDist);
					auton.Rotate(90);
					auton.Drive(SpeedFast, scAlley);
					auton.Rotate(-90);
					auton.Drive(SpeedSlow, scDistFinal);
					ele.PushLo();

				break;

				case 7 :
					//RSCR
					SmartDashboard::PutString("Auton Info", "Robot on right, going for scale on right");
					auton.Drive(SpeedFast, scDist);
					auton.Rotate(90);
					auton.Drive(SpeedSlow, scApproachDist);
					ele.PushLo();

				break;

				default :
					//Just Go FWD
					SmartDashboard::PutString("Auton Info", "No Auton, Just going fwd and stoping");
						auton.Drive(SpeedFast, 30);
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

		while (IsOperatorControl() && IsEnabled()) {

			if(joy.GetRawButton(8)){
				eleCam.SetFPS(0);
				climbCam.SetFPS(15);
			}else if(joy.GetRawButton(7)){
				eleCam.SetFPS(15);
				climbCam.SetFPS(0);
			}
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

			if(eject.isPressed())
				ele.PushLo();

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

			//ele.Move(joy2.GetRawAxis(1));

			drive.Drive(deadzone(joy.GetRawAxis(1)), deadzone(joy.GetRawAxis(4)), true);

			//ele.Refresh();

			if(joy2.GetRawButton(7))
				climber.Set(frc::DoubleSolenoid::Value::kForward);
			else if(joy2.GetRawButton(8))
				climber.Set(frc::DoubleSolenoid::Value::kReverse);

			SmartDashboard::PutNumber("Left Encoder Vel", drive.GetEncVel()[0]);
			SmartDashboard::PutNumber("Right Encoder Vel", drive.GetEncVel()[1]);
			//SmartDashboard::PutNumber("Total Amps", pdp.GetTotalCurrent());
			//SmartDashboard::PutNumber("Total Power", pdp.GetTotalPower());

			frc::Wait(0.005);
		}
	}
	void Test() override {}

};

START_ROBOT_CLASS(Robot)
