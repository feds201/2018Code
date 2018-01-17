
#include"WPILib.h"
#include"ctre/Phoenix.h"

class Robot : public frc::SampleRobot {

public:
	Robot() {

	}

	void RobotInit() {

	}

	void Autonomous() {

	}

	void OperatorControl() override {

		while (IsOperatorControl() && IsEnabled()) {

			frc::Wait(0.005);
		}
	}
	void Test() override {}

};

START_ROBOT_CLASS(Robot)
