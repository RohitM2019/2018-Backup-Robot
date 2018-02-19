#include <ctre/Phoenix.h>
#include "ahrs.h"
#include <cmath>

#ifndef SRC_SFDRIVE_H_
#define SRC_SFDRIVE_H_

class SFDrive
{
    public:
        //MEMBER VARIABLES
        WPI_TalonSRX * m_leftMotor;
        WPI_TalonSRX * m_rightMotor;
        AHRS * m_ahrs;
        double m_deadband = 0.08;

    public:
        SFDrive (WPI_TalonSRX * lMotor, WPI_TalonSRX * rMotor, AHRS * ahrs = NULL);
        void ArcadeDrive (double xSpeed, double zRotation);
};

#endif
