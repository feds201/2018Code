
#include"WPILib.h"
#include"ctre/Phoenix.h"
#include"Drivetrain.h"
#include"Auton.h"
#include"EdgeDetection.h"

class Robot : public frc::SampleRobot {

	Drivetrain drive;
	Joystick joy;
	Compressor comp;
	Auton auton;
	Edge shift;
	WPI_TalonSRX m1;
	WPI_TalonSRX m2;

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

	}

	void Autonomous() {

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
