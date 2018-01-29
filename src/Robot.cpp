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
	const double scale = 1;
private:
	WPI_TalonSRX * _rghtFront = new WPI_TalonSRX(rMotorNum);
	WPI_TalonSRX * _rghtFollower = new WPI_TalonSRX(lMotorNum);

	DifferentialDrive *myRobot = new DifferentialDrive(*_rghtFront,*_rghtFollower );
	Joystick *stick = new Joystick(joystickNum);
	
	Faults _faults_L;
	Faults _faults_R;
	
	WPI_TalonSRX *leftMotor = new WPI_TalonSRX(lMotorNum);

	void RobotInit()//Robot-wide initialization code should go here.
	{
		ctre::phoenix::motorcontrol::FeedbackDevice qE = QuadEncoder;
		leftMotor->ConfigSelectedFeedbackSensor(qE,0,0);
	}
	/*
	void RobotEncoder() {
		if (leftMotor->BaseMotorController::GetSelectedSensorPosition() == 0)
		{
			//do a thing
		}
	}
	*/

	void TeleopInit()//Initialization code for teleop mode should go here.
	{
		myRobot->ArcadeDrive(0.0, 0.0);
	}


	void TeleopPeriodic()//Periodic code for teleop mode should go here.
	{
		myRobot->ArcadeDrive(scale * stick->GetRawAxis(1), (stick->GetRawAxis(4) > 0? 1:-1) * stick->GetRawAxis(4) * stick->GetRawAxis(4));
	}

	void AutonomousPeriodic()//Periodic code for autonomous mode should go here.
	{
		//rMotor->Set(ControlMode::Current, 1);
		//lMotor->Set(ControlMode::Current, 1);
	}
};

START_ROBOT_CLASS(Robot)
