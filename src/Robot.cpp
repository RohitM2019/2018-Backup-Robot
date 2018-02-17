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

class Robot : public frc::IterativeRobot
{

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
        double setPoint = 0;
        //the timeout that you want to set.(If zero, no blocking or checking is performed.)
        int checkTimeout = 0;
        int packetsReceived = 0;

    private:
        WPI_TalonSRX * _rMotor = new WPI_TalonSRX (rMotorNum);
        WPI_TalonSRX * _lMotor = new WPI_TalonSRX (lMotorNum);

        SFDrive *myRobot = new SFDrive (_lMotor, _rMotor);
        Joystick *stick = new Joystick (joystickNum);

        //the error for the PIDs
        double r_error = (TICKS_PER_INCH * inches) - (_rMotor->GetSelectedSensorPosition (0));
        double l_error = (TICKS_PER_INCH * inches) - (_lMotor->GetSelectedSensorPosition (0));

        int countms = 0;

        //used for setting the PIDs in teleop
        double setpoint = 0;

        void RobotInit ()
        {
            //used to config the motor controllers for QuadEncoders(type of encoder)
            ctre::phoenix::motorcontrol::FeedbackDevice qE = QuadEncoder;
            _lMotor->ConfigSelectedFeedbackSensor (qE, 0, checkTimeout);
            _rMotor->ConfigSelectedFeedbackSensor (qE, 0, checkTimeout);

            //used for inverting motors
            _rMotor->SetSensorPhase (true);
            _lMotor->SetSensorPhase (true);

            //adds PIDs to shuffle board
            frc::SmartDashboard::PutNumber ("P", pConstant);
            frc::SmartDashboard::PutNumber ("I", iConstant);
            frc::SmartDashboard::PutNumber ("D", dConstant);

        }

        void AutonomousInit ()
        {
            //config the PIDs
            /*
             * (done) 1. Start testing 0 for kI and kD
             * 2. Set kP until robot shakes
             * 3. increase kD until it stops isolating
             * 4. set kI for going up ramps
             */
            DriverStation::ReportError ("AutonInit Started");
            ConfigPIDS ();
            DriverStation::ReportError ("AutonInit Completed");
        }

        void AutonomousPeriodic ()
        {

        }

        void TestInit ()
        {
            DriverStation::ReportError ("TestInit Started");
            ConfigPIDS ();
            //Put PID values in ShuffleBoard
            SmartDashboard::PutNumber ("P Drive", pConstant);
            SmartDashboard::PutNumber ("I Drive", iConstant);
            SmartDashboard::PutNumber ("D Drive", dConstant);
            SmartDashboard::PutNumber ("Setpoint Drive", 0);
            SmartDashboard::PutNumber ("Current Position - Right", 0);
            SmartDashboard::PutNumber ("Current Position - Left", 0);
            DriverStation::ReportError ("TestInit Completed");
        }

        void TestPeriodic ()
        {
            if (packetsReceived % 100 == 0) //Update PID and setpoint values from shuffleboard
            {
                //Every 100 packets (2 seconds), update P, I, D values
                pConstant = SmartDashboard::GetNumber ("P Drive", pConstant);
                iConstant = SmartDashboard::GetNumber ("I Drive", iConstant);
                dConstant = SmartDashboard::GetNumber ("D Drive", dConstant);
                setPoint = SmartDashboard::GetNumber ("Setpoint Drive", setPoint);
                SmartDashboard::PutNumber ("Current Position - Right", _rMotor->GetSensorCollection ().GetQuadraturePosition ());
                SmartDashboard::PutNumber ("Current Position - Left", _lMotor->GetSensorCollection ().GetQuadraturePosition ());
                _lMotor->Config_kP (0, pConstant, checkTimeout);
                _lMotor->Config_kI (0, iConstant, checkTimeout);
                _lMotor->Config_kD (0, dConstant, checkTimeout);
                _rMotor->Config_kP (0, pConstant, checkTimeout);
                _rMotor->Config_kI (0, iConstant, checkTimeout);
                _rMotor->Config_kD (0, dConstant, checkTimeout);
            }

            packetsReceived++;
            myRobot->PIDDrive (setPoint, setPoint);
        }

        void ConfigPIDS ()
        {
            DriverStation::ReportError ("PID Config Started");
            _rMotor->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);
            _rMotor->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);
            _lMotor->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);
            _lMotor->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);

            _lMotor->Config_kP (0, pConstant, checkTimeout);
            _lMotor->Config_kI (0, iConstant, checkTimeout);
            _lMotor->Config_kD (0, dConstant, checkTimeout);

            _rMotor->Config_kP (0, pConstant, checkTimeout);
            _rMotor->Config_kI (0, iConstant, checkTimeout);
            _rMotor->Config_kD (0, dConstant, checkTimeout);

            DriverStation::ReportError ("PID Config Completed");
        }
};

START_ROBOT_CLASS(Robot)
