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
        //PID variables. TODO - FIGURE OUT WHY THE HELL THIS WORKS
        double pConstant = 1;
        double iConstant = 0;
        double dConstant = 0;
        double maxAccel = 100;
        double setPoint = 0;
        //the timeout that you want to set.(If zero, no blocking or checking is performed.)
        int checkTimeout = 0;
        int packetsReceived = 0;
        ctre::phoenix::motion::MotionProfileStatus autonStatus;

    private:
        WPI_TalonSRX * _rMotor = new WPI_TalonSRX (rMotorNum);
        WPI_TalonSRX * _lMotor = new WPI_TalonSRX (lMotorNum);

        SFDrive *myRobot = new SFDrive (_lMotor, _rMotor);
        Joystick *stick = new Joystick (joystickNum);

        void RobotInit ()
        {
            //used to config the motor controllers for QuadEncoders(type of encoder)
            ctre::phoenix::motorcontrol::FeedbackDevice qE = QuadEncoder;
            _lMotor->ConfigSelectedFeedbackSensor (qE, 0, checkTimeout);
            _rMotor->ConfigSelectedFeedbackSensor (qE, 0, checkTimeout);

            //used for inverting motors
            _rMotor->SetInverted(true);
            _lMotor->SetInverted(true);
            _rMotor->SetSensorPhase(true);
            _lMotor->SetSensorPhase(true);

            _rMotor->ConfigMaxIntegralAccumulator(0, 0.1, checkTimeout);
            _lMotor->ConfigMaxIntegralAccumulator(0, 0.1, checkTimeout);

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
            TrajectoryPoint first;
            TrajectoryPoint second;

            first.position = 100000;
            second.position = 0;

            _lMotor->PushMotionProfileTrajectory(first);
            _lMotor->PushMotionProfileTrajectory(second);
            _lMotor->ProcessMotionProfileBuffer();
            DriverStation::ReportError ("AutonInit Completed");
        }

        void AutonomousPeriodic ()
        {
            _lMotor->GetMotionProfileStatus(autonStatus);
            if(!autonStatus.activePointValid)
            _lMotor->ProcessMotionProfileBuffer();
        }

        void TeleopInit ()
        {
            myRobot->ArcadeDrive (0.0, 0.0);
        }

        void TeleopPeriodic ()
        {
            myRobot->ArcadeDrive (scale * stick->GetRawAxis (1), -(stick->GetRawAxis (4) > 0 ? 1 : -1) * stick->GetRawAxis (4) * stick->GetRawAxis (4));
        }

        void TestInit ()
        {
            DriverStation::ReportError ("TestInit Started");
            ConfigPIDS ();
            //Put PID values in ShuffleBoard
            SmartDashboard::PutNumber ("P Drive", pConstant);
            SmartDashboard::PutNumber ("I Drive", iConstant);
            SmartDashboard::PutNumber ("D Drive", dConstant);
            SmartDashboard::PutNumber("Maximum Acceleration", maxAccel);
            SmartDashboard::PutNumber ("Setpoint Drive", 0);
            SmartDashboard::PutNumber ("Current Position - Right", 0);
            SmartDashboard::PutNumber ("Current Position - Left", 0);
            SmartDashboard::PutNumber("lMotor Profile", 0);
            SmartDashboard::PutNumber("rMotor Profile", 0);
            SmartDashboard::PutBoolean("Setpoint?", false);
            DriverStation::ReportError ("TestInit Completed");
        }

        void TestPeriodic ()
        {
            //FIX IF STATEMENT TO USE TIMESTAMP
            if (packetsReceived % 10 == 0) //Update PID and setpoint values from shuffleboard
            {
                //Every 100 packets (2 seconds), update P, I, D values
                pConstant = SmartDashboard::GetNumber ("P Drive", pConstant);
                iConstant = SmartDashboard::GetNumber ("I Drive", iConstant);
                dConstant = SmartDashboard::GetNumber ("D Drive", dConstant);
                maxAccel = SmartDashboard::GetNumber("Maximum Acceleration", maxAccel);
                setPoint = SmartDashboard::GetNumber ("Setpoint Drive", setPoint);
                SmartDashboard::PutNumber ("Current Position - Right", _rMotor->GetSensorCollection ().GetQuadraturePosition ());
                SmartDashboard::PutNumber ("Current Position - Left", _lMotor->GetSensorCollection ().GetQuadraturePosition ());
                _lMotor->Config_kP (0, pConstant, checkTimeout);
                _lMotor->Config_kI (0, iConstant, checkTimeout);
                _lMotor->Config_kD (0, dConstant, checkTimeout);
                _lMotor->ConfigMotionAcceleration(maxAccel, checkTimeout);
                _rMotor->Config_kP (0, pConstant, checkTimeout);
                _rMotor->Config_kI (0, iConstant, checkTimeout);
                _rMotor->Config_kD (0, dConstant, checkTimeout);
                _rMotor->ConfigMotionAcceleration(maxAccel, checkTimeout);
            }

            _rMotor->Set(ctre::phoenix::motorcontrol::ControlMode::MotionProfile, 1);
            _lMotor->Set(ctre::phoenix::motorcontrol::ControlMode::MotionProfile, 1);

            packetsReceived++;
            if(SmartDashboard::GetBoolean("Setpoint?", false))
            {
                TrajectoryPoint l;
                TrajectoryPoint r;
                l.position = -setPoint;
                r.position = setPoint;
                l.velocity = 0.8;
                r.velocity = 0.8;
                l.zeroPos = false;
                r.zeroPos = false;

                _lMotor->PushMotionProfileTrajectory(l);
                _rMotor->PushMotionProfileTrajectory(r);
                _lMotor->ProcessMotionProfileBuffer();
                _rMotor->ProcessMotionProfileBuffer();
                SmartDashboard::PutBoolean("Setpoint?", false);
                DriverStation::ReportError(std::to_string(_lMotor->GetMotionProfileTopLevelBufferCount()));
            }


            //DriverStation::ReportError(std::to_string(_lMotor->));
            //myRobot->PIDDrive (setPoint, -setPoint);

        }

        void ConfigPIDS ()
        {
            _rMotor->Set(ctre::phoenix::motorcontrol::ControlMode::MotionProfile, 1);
            _lMotor->Set(ctre::phoenix::motorcontrol::ControlMode::MotionProfile, 1);

            _rMotor->ClearMotionProfileTrajectories();
            _lMotor->ClearMotionProfileTrajectories();

            DriverStation::ReportError ("PID Config Started");
            _rMotor->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);
            _rMotor->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);
            _lMotor->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);
            _lMotor->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);

            _lMotor->Config_kP (0, pConstant, checkTimeout);
            _lMotor->Config_kI (0, iConstant, checkTimeout);
            _lMotor->Config_kD (0, dConstant, checkTimeout);
            _lMotor->ConfigMotionAcceleration(maxAccel, checkTimeout);
            _rMotor->Config_kP (0, pConstant, checkTimeout);
            _rMotor->Config_kI (0, iConstant, checkTimeout);
            _rMotor->Config_kD (0, dConstant, checkTimeout);
            _rMotor->ConfigMotionAcceleration(maxAccel, checkTimeout);

            DriverStation::ReportError ("PID Config Completed");
        }
};

START_ROBOT_CLASS(Robot)
