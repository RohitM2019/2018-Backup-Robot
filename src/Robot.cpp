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
	WPI_TalonSRX * _rMotor = new WPI_TalonSRX(rMotorNum);
	WPI_TalonSRX * _lMotor = new WPI_TalonSRX(lMotorNum);

	DifferentialDrive *myRobot = new DifferentialDrive(*_rMotor,*_lMotor );
	Joystick *stick = new Joystick(joystickNum);

	void RobotInit()
	{
		ctre::phoenix::motorcontrol::FeedbackDevice qE = QuadEncoder;
		_lMotor->ConfigSelectedFeedbackSensor(qE,0,0);
	}
	
	void AutonomousInit(){
		_rMotor -> Set(ctre::phoenix::motorcontrol::ControlMode::Position,27600);
		//20 feet
	}


	void TeleopInit()
	{
		myRobot->ArcadeDrive(0.0, 0.0);
	}


	void TeleopPeriodic()
	{
		myRobot->ArcadeDrive(scale * stick->GetRawAxis(1), (stick->GetRawAxis(4) > 0? 1:-1) * stick->GetRawAxis(4) * stick->GetRawAxis(4));
	}

	void AutonomousPeriodic()//Periodic code for autonomous mode should go here.
	{
		//WPI_TalonSRX::WPI_TalonSRX (2);
		//void WPI_TalonSRX::Set(ctre::phoenix::motorcontrol::ControlMode::Position,27600);
		//20 feet

		//rMotor->Set(ControlMode::Current, 1);
		//lMotor->Set(ControlMode::Current, 1);
		if (_lMotor->BaseMotorController::GetSelectedSensorPosition(0) == 4)//4 is a test value, effect may not be noticeable.
		{
			//do a thing
			
		}


	}
};

START_ROBOT_CLASS(Robot)
