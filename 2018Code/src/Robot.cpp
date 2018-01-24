
#include"WPILib.h"
#include"ctre/Phoenix.h"
#include"Drivetrain.h"
#include"Auton.h"
#include"EdgeDetection.h"
#include <SmartDashboard/SendableChooser.h>
#include <iostream>

class Robot : public frc::SampleRobot {

	Drivetrain drive;
	Joystick joy;
	Compressor comp;
	Auton auton;
	Edge shift;
	WPI_TalonSRX m1;
	WPI_TalonSRX m2;
	frc::SendableChooser<std::string> chooser;
	frc::SendableChooser<std::string> lrchooser;

public:
	Robot():

	drive(3, 4, 5, 7, 6, 8, 0, 1), joy(0), comp(8), auton(&drive), shift(joy.GetRawButton(1)), m1(1), m2(2)

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

	}

	void Autonomous() {

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

				break;

				case 1 :
					//LSR
					SmartDashboard::PutString("Auton Info", "Robot on left, going for switch on right");

				break;

				case 2 :
					//LSCL
					SmartDashboard::PutString("Auton Info", "Robot on left, going for scale on left");

				break;

				case 3 :
					//LSCR
					SmartDashboard::PutString("Auton Info", "Robot on left, going for scale on right");

				break;

				case 4 :
					//RSL
					SmartDashboard::PutString("Auton Info", "Robot on right, going for switch on left");

				break;

				case 5 :
					//RSR
					SmartDashboard::PutString("Auton Info", "Robot on right, going for switch on right");

				break;

				case 6 :
					//RSCL
					SmartDashboard::PutString("Auton Info", "Robot on right, going for scale on left");

				break;

				case 7 :
					//RSCR
					SmartDashboard::PutString("Auton Info", "Robot on right, going for scale on right");

				break;

				default :
					//Just Go FWD
					SmartDashboard::PutString("Auton Info", "No Auton, Just going fwd and stoping");

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
		frc::Wait(4);
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

			shift.update(joy.GetRawButton(1));

			if(shift.isPressed())
				drive.Shift();

			drive.Drive(deadzone(joy.GetRawAxis(1)), deadzone(joy.GetRawAxis(4)), true);

			frc::Wait(0.005);
		}
	}
	void Test() override {}

};

START_ROBOT_CLASS(Robot)
