
#include"WPILib.h"
#include"ctre/Phoenix.h"
#include"Drivetrain.h"

class Robot : public frc::SampleRobot {

	Drivetrain drive;
	Joystick joy;


public:
	Robot():

		drive(1, 2, 4, 3, 8, 0, 1, 6), joy(0)

	{


	}

	void RobotInit() {

	}

	void Autonomous() {

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

		while (IsOperatorControl() && IsEnabled()) {

			drive.Drive(deadzone(joy.GetRawAxis(1)), deadzone(joy.GetRawAxis(4)), true);

			frc::Wait(0.005);
		}
	}
	void Test() override {}

};

START_ROBOT_CLASS(Robot)
