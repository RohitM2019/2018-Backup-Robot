/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>
#include <Joystick.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <DriverStation.h>
#include <MotorSafetyHelper.h>
#include "SFDrive.h"

class Robot: public frc::IterativeRobot {
public:
	const int joystickNum = 0;
	const int rMotorNum = 2;
	const int lMotorNum = 6;
	const double scale = 1;
	double pConstant = 1.0/400.0;
	double iConstant = 0.00;
	double dConstant = 0.00;
	int checkTimeout = 0;
	const double TICKS_PER_INCH = 325.95;
private:
	WPI_TalonSRX * _rMotor = new WPI_TalonSRX(rMotorNum);
	WPI_TalonSRX * _lMotor = new WPI_TalonSRX(lMotorNum);


	SFDrive *myRobot = new SFDrive(_lMotor, _rMotor );
	Joystick *stick = new Joystick(joystickNum);

	void RobotInit()
	{
	//used to config the motor controllers for QuadEncoders(type of encoder)
		ctre::phoenix::motorcontrol::FeedbackDevice qE = QuadEncoder;
		//2nd arg = Primary closed-loop(If zero, no blocking or checking is performed.)
		//Primary closed-loop =
		_lMotor->ConfigSelectedFeedbackSensor(qE,0,checkTimeout);
		_rMotor->ConfigSelectedFeedbackSensor(qE,0,checkTimeout);
		_rMotor->SetSensorPhase(true);
	}
	
	void AutonomousInit()
	{
	//config the PIDs

		/*
		 * 1. Start testing 0 for kI and kD
		 * 2. Set kP until robot shakes
		 * 3. increase kD until it stops isolating
		 * 4. set kI for going up ramps
		 */

			//slotIdx = Which CAN bus
		_lMotor->Config_kP(0,pConstant,checkTimeout);
		_lMotor->Config_kI(0,iConstant,checkTimeout);
		_lMotor->Config_kD(0,dConstant,checkTimeout);
		//sets the encoder value
		_lMotor->GetSensorCollection().SetQuadraturePosition(0, checkTimeout);

		_rMotor->Config_kP(0,pConstant,checkTimeout);
		_rMotor->Config_kI(0,iConstant,checkTimeout);
		_rMotor->Config_kD(0,dConstant,checkTimeout);
		//sets the encoder value
		_rMotor->GetSensorCollection().SetQuadraturePosition(0, checkTimeout);

		//sets the safety (need this for FRC)
		_rMotor->SetSafetyEnabled(true);
		_lMotor->SetSafetyEnabled(true);

		//Selects which CAN you are using
		_rMotor->SelectProfileSlot(0,0);
		_lMotor->SelectProfileSlot(0,0);
	}



	void TeleopInit()
	{
		myRobot->ArcadeDrive(0.0, 0.0);

		//sets the encoder value to 0 to begin with
		_lMotor->GetSensorCollection().SetQuadraturePosition(0, checkTimeout);
		_rMotor->GetSensorCollection().SetQuadraturePosition(0, checkTimeout);

	}


	void TeleopPeriodic()
	{
		//push more up/down/left/right on the controller the faster the robot moves
		myRobot->ArcadeDrive(scale * stick->GetRawAxis(1), (stick->GetRawAxis(4) > 0? 1:-1) * stick->GetRawAxis(4) * stick->GetRawAxis(4));

		//reports the error for the selected sensor position( 0 = Primary closed-loop)
		DriverStation::ReportError(std::to_string(_rMotor->GetSelectedSensorPosition(0)));
		DriverStation::ReportError(std::to_string(_lMotor->GetSelectedSensorPosition(0)));
	}

	void AutonomousPeriodic()
	{
		//Should be in Init
		_rMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, TICKS_PER_INCH * 12 );
		_lMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, TICKS_PER_INCH * 12 );

		//DriverStation::ReportError(std::to_string(_rMotor->GetSelectedSensorPosition(0)) + " Right Control Mode:: " + std::to_string((int) _rMotor->GetControlMode()));
		//DriverStation::ReportError(std::to_string(_lMotor->GetSelectedSensorPosition(0)) + " Left  Control Mode:: " + std::to_string((int) _lMotor->GetControlMode()));
	}
};

START_ROBOT_CLASS(Robot)
