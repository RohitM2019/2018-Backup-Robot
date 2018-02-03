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
#include <DriverStation.h>
#include <MotorSafetyHelper.h>

class Robot: public frc::IterativeRobot {
public:
	const int joystickNum = 0;
	const int rMotorNum = 2;
	const int lMotorNum = 6;
	const double scale = 1;
	double pConstant = 1.00;
	double iConstant = 0.00;
	double dConstant = 0.00;
private:
	WPI_TalonSRX * _rMotor = new WPI_TalonSRX(rMotorNum);
	WPI_TalonSRX * _lMotor = new WPI_TalonSRX(lMotorNum);

	DifferentialDrive *myRobot = new DifferentialDrive(*_lMotor,*_rMotor );
	Joystick *stick = new Joystick(joystickNum);

	void RobotInit()
	{
		ctre::phoenix::motorcontrol::FeedbackDevice qE = QuadEncoder;
		_lMotor->ConfigSelectedFeedbackSensor(qE,0,2);
		_rMotor->ConfigSelectedFeedbackSensor(qE,0,2);
		_rMotor->SetSensorPhase(true);

		_rMotor -> SetSafetyEnabled(false);
		_lMotor -> SetSafetyEnabled(false);
	}
	
	void AutonomousInit()
	{
		_lMotor->Config_kP(0,pConstant,2);
		_lMotor->Config_kD(0,dConstant,2);
		_lMotor->Config_kI(0,iConstant,2);
		_lMotor->GetSensorCollection().SetQuadraturePosition(0, 2);

		_rMotor->Config_kP(0,pConstant,2);
		_rMotor->Config_kD(0,dConstant,2);
		_rMotor->Config_kI(0,iConstant,2);
		_rMotor->GetSensorCollection().SetQuadraturePosition(0, 2);

	}


	void TeleopInit()
	{
		myRobot->ArcadeDrive(0.0, 0.0);
	}


	void TeleopPeriodic()
	{
		myRobot->ArcadeDrive(scale * stick->GetRawAxis(1), (stick->GetRawAxis(4) > 0? 1:-1) * stick->GetRawAxis(4) * stick->GetRawAxis(4));
		DriverStation::ReportError(std::to_string(_rMotor->GetSelectedSensorPosition(0)));
	}

	void AutonomousPeriodic()
	{
		_rMotor -> Set(ctre::phoenix::motorcontrol::ControlMode::Position,6875.4);
		_lMotor -> Set(ctre::phoenix::motorcontrol::ControlMode::Position,6875.4);
		//Periodic code for autonomous mode should go here.
		//114.59 ticks = 1 in for 4" wheel diamiter
		//WPI_TalonSRX::WPI_TalonSRX (2);
		//void WPI_TalonSRX::Set(ctre::phoenix::motorcontrol::ControlMode::Position,27600);
		//20 feet
		//_rMotor->Set(ControlMode::Current, 1);
		//_lMotor->Set(ControlMode::Current, 1);
		//if (_lMotor-> Get() == 0){
			//this should test for position, not velocity, or you'd never start to begin with
		//}
		//else{
			//
			
		//}
	}
};

START_ROBOT_CLASS(Robot)
