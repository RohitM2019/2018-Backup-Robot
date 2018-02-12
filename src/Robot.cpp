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
	//used for auton distance
	int inches = 60;
	const double TICKS_PER_INCH = 325.95;
	//PID variables
	double pConstant = .1;
	double iConstant = 0.001;
	double dConstant = 0;
	//the timeout that you want to set.(If zero, no blocking or checking is performed.)
	int checkTimeout = 0;

private:
	WPI_TalonSRX * _rMotor = new WPI_TalonSRX(rMotorNum);
	WPI_TalonSRX * _lMotor = new WPI_TalonSRX(lMotorNum);

	SFDrive *myRobot = new SFDrive(_lMotor, _rMotor );
	Joystick *stick = new Joystick(joystickNum);

	//the error for the PIDs
	double r_error = (TICKS_PER_INCH * inches)-(_rMotor->GetSelectedSensorPosition(0));
	double l_error = (TICKS_PER_INCH * inches)-(_lMotor->GetSelectedSensorPosition(0));

	int countms = 0;

	//used for setting the PIDs in teleop
	double setpoint = 0;


	void RobotInit()
	{
	//used to config the motor controllers for QuadEncoders(type of encoder)
		ctre::phoenix::motorcontrol::FeedbackDevice qE = QuadEncoder;
		_lMotor->ConfigSelectedFeedbackSensor(qE,0,checkTimeout);
		_rMotor->ConfigSelectedFeedbackSensor(qE,0,checkTimeout);

		//used for inverting motors
		_rMotor->SetSensorPhase(true);
		_lMotor->SetSensorPhase(true);

		//adds PIDs to shuffle board
		frc::SmartDashboard::PutNumber("P", pConstant);
		frc::SmartDashboard::PutNumber("I", iConstant);
		frc::SmartDashboard::PutNumber("D", dConstant);

	}
	

	void AutonomousInit()
	{
	//config the PIDs
		/*
		 * (done) 1. Start testing 0 for kI and kD
		 * 2. Set kP until robot shakes
		 * 3. increase kD until it stops isolating
		 * 4. set kI for going up ramps
		 */

		//inverts the motors in Auton
		_lMotor->SetInverted(true);
		_rMotor->SetInverted(true);
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
		_rMotor->SetSafetyEnabled(false);
		_lMotor->SetSafetyEnabled(false);

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

		//inverts the motors in teleop
		_lMotor->SetInverted(true);
		_rMotor->SetInverted(true);
	}


	void TeleopPeriodic()
	{
		//push more up/down/left/right on the controller the faster the robot moves
		//myRobot->ArcadeDrive(scale * stick->GetRawAxis(1), (stick->GetRawAxis(4) > 0? 1:-1) * stick->GetRawAxis(4) * stick->GetRawAxis(4));

		//if you release A button move one rotation forward(used to set the PIDs
		if(stick->GetRawButtonReleased(1)){
			setpoint += 4096.001;
		}
		//if you release B button move one rotation backwards
		if(stick->GetRawButtonReleased(2)){
					setpoint -= 4096.001;
		}
		_rMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position,-(setpoint));
		_lMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position,setpoint);

		//reports the encoder values of Right and Left motors
		DriverStation::ReportError("Right Motor: " + std::to_string(_rMotor->GetSelectedSensorPosition(0)) + " Left Motor: " + std::to_string(_lMotor->GetSelectedSensorPosition(0)));

		//reports what values the PIDs are
		DriverStation::ReportError("right: " + std::to_string(_rMotor->GetSelectedSensorPosition(0)) + "  left: " + std::to_string(_lMotor->GetSelectedSensorPosition(0)));
		DriverStation::ReportError("D: " +
			std::to_string(_lMotor->ConfigGetParameter(eProfileParamSlot_D,0,checkTimeout)) +
			"I: " +
			std::to_string(_lMotor->ConfigGetParameter(eProfileParamSlot_I,0,checkTimeout)) +
			"P: " +
			std::to_string(_lMotor->ConfigGetParameter(eProfileParamSlot_P,0,checkTimeout)));
	}

	void AutonomousPeriodic()
	{
		//AutonomousPeriodic gets called every 50ms so 50 x 6 = 300ms
			//every 300ms reports the error of the left and right motor
		countms = countms + 1;
		if(countms == 6){
			countms = 0;
			//DriverStation::ReportError("left: " + std::to_string(l_error));
		}

		_rMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position,-(TICKS_PER_INCH* inches));
		_lMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position,TICKS_PER_INCH* inches);

		//_rMotor->_safetyHelper.ClearError();
		//DriverStation::ReportError(std::to_string(_rMotor->GetSelectedSensorPosition(0)) + " Right Control Mode:: " + std::to_string((int) _rMotor->GetControlMode()));
		//DriverStation::ReportError(std::to_string(_lMotor->GetSelectedSensorPosition(0)) + " Left  Control Mode:: " + std::to_string((int) _lMotor->GetControlMode()));
	}
};

START_ROBOT_CLASS(Robot)
