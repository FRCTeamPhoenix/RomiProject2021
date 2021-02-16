#pragma once

#include "Constants.h"
#include "sensors/RomiGyro.h"

#include <frc2/command/SubsystemBase.h>
#include <frc/Spark.h>
#include <frc/Encoder.h>
#include <frc/drive/DifferentialDrive.h>
#include <units/length.h>

class Drivetrain : public frc2::SubsystemBase{
public: 
    Drivetrain();

    void ArcadeDrive(double moveX, double rotZ);
    void TankDrive(double left, double right);

    units::meter_t GetLeftDistance();
    units::meter_t GetRightDistance();
    units::meter_t GetAverageDistance();

    //resets distance to zero on both encoders
    void ZeroEncoders();

    RomiGyro* GetGyro() { return &m_gyro; }
private:
    RomiGyro m_gyro;

    frc::Spark m_leftMotor{MOTOR_LEFT};
    frc::Spark m_rightMotor{MOTOR_RIGHT};

    frc::Encoder m_leftEncoder{4, 5};
    frc::Encoder m_rightEncoder{6, 7};

    frc::DifferentialDrive m_drive{m_leftMotor, m_rightMotor};
};