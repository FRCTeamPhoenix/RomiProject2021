#pragma once

#include "Constants.h"
#include "sensors/RomiGyro.h"

#include <frc2/command/SubsystemBase.h>
#include <frc/Spark.h>
#include <frc/Encoder.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <units/length.h>

class Drivetrain : public frc2::SubsystemBase{
public: 
    Drivetrain();

    //called periodically to update pose
    void Periodic() override;

    //basic arcade drive
    void ArcadeDrive(double moveX, double rotZ);
    //curvature drive designed for turning while moving (handles turn in place if xSpeed < 0.005)
    void CurvatureDrive(double moveX, double rotZ);

    void TankDrive(double left, double right);
    void DriveVolts(units::volt_t left, units::volt_t right);

    frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();
    frc::Pose2d GetPose();
    units::meter_t GetLeftDistance();
    units::meter_t GetRightDistance();
    units::meter_t GetAverageDistance();
    bool IsStopped();

    //resets distance to zero on both encoders
    void ZeroEncoders();
    //sets the current position to be pose
    void ResetOdometry(frc::Pose2d pose);

    RomiGyro* GetGyro() { return &m_gyro; }
private:
    RomiGyro m_gyro;

    frc::Spark m_leftMotor{MOTOR_LEFT};
    frc::Spark m_rightMotor{MOTOR_RIGHT};

    frc::Encoder m_leftEncoder{4, 5};
    frc::Encoder m_rightEncoder{6, 7};

    frc::DifferentialDrive m_drive{m_leftMotor, m_rightMotor};
    frc::DifferentialDriveOdometry m_odometry{units::degree_t(m_gyro.GetAngleZ())};
};