/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>
#include <Drive/DifferentialDrive.h>
#include <Joystick.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>



class Robot: public frc::IterativeRobot {
public:
	const int joystickNum = 0;
	const int rMotorNum = 2;
	const int lMotorNum = 6;
private:
	TalonSRX *rMotor = new TalonSRX(rMotorNum);
	TalonSRX *lMotor = new TalonSRX(lMotorNum);

<<<<<<< HEAD
	//DifferentialDrive *myRobot = new DifferentialDrive((SpeedController&) rMotor,(SpeedController&) lMotor );
=======
	DifferentialDrive *myRobot = new DifferentialDrive((SpeedController&) lMotor,(SpeedController&) rMotor );
>>>>>>> origin/master
	Joystick *stick = new Joystick(joystickNum);
	void RobotInit() {

	}

	void TeleopInit() {
<<<<<<< HEAD
		//myRobot->ArcadeDrive(0, 0);
=======
		myRobot->ArcadeDrive(1.0, 1.0,true);
>>>>>>> origin/master
	}

	void TeleopPeriodic() {
		//myRobot->ArcadeDrive(stick->GetY(), stick->GetX());

		myRobot->ArcadeDrive(1.0, 1.0);
	}

	void AutonomousPeriodic() {
		//rMotor->Set(ControlMode::Current, 1);
		//lMotor->Set(ControlMode::Current, 1);
	}
};

START_ROBOT_CLASS(Robot)
