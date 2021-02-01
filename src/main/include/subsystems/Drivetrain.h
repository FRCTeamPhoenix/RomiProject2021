#pragma once

#include "Constants.h"

#include <frc2/command/SubsystemBase.h>
#include <frc/Spark.h>
#include <frc/drive/DifferentialDrive.h>

class Drivetrain : public frc2::SubsystemBase{
public: 
    Drivetrain();

    void ArcadeDrive(float moveX, float rotZ);
private:
    frc::Spark m_leftMotor{MOTOR_LEFT};
    frc::Spark m_rightMotor{MOTOR_RIGHT};

    frc::DifferentialDrive m_drive{m_leftMotor, m_rightMotor};
};