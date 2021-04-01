#include "subsystems/Drivetrain.h"

#include <wpi/math>
#include <frc/smartdashboard/SmartDashboard.h>

Drivetrain::Drivetrain(){
    m_leftEncoder.SetDistancePerPulse(wpi::math::pi * WHEEL_DIAMETER.to<double>() / TICKS_PER_REVOLUTION);
    m_rightEncoder.SetDistancePerPulse(wpi::math::pi * WHEEL_DIAMETER.to<double>() / TICKS_PER_REVOLUTION);

    //set the speed thresholds
    m_leftEncoder.SetMinRate(MIN_STOP_SPEED.to<double>());
    m_rightEncoder.SetMinRate(MIN_STOP_SPEED.to<double>());

    m_gyro.Reset();
    ZeroEncoders();
}

void Drivetrain::Periodic(){
    m_odometry.Update(frc::Rotation2d(units::degree_t(-m_gyro.GetAngleZ())),
        units::meter_t(m_leftEncoder.GetDistance()), 
        units::meter_t(m_rightEncoder.GetDistance()));

    frc::SmartDashboard::PutNumber("Pose Angle", GetPose().Rotation().Degrees().to<double>());
    frc::SmartDashboard::PutNumber("Pose X", GetPose().X().to<double>());
    frc::SmartDashboard::PutNumber("Pose Y", GetPose().Y().to<double>());
}

void Drivetrain::ArcadeDrive(double moveX, double rotZ){
    m_drive.ArcadeDrive(moveX, rotZ);
}

void Drivetrain::CurvatureDrive(double moveX, double rotZ){
    m_drive.CurvatureDrive(moveX, rotZ, moveX < 0.1);
}

void Drivetrain::TankDrive(double left, double right){
    m_drive.TankDrive(left, right);
}

void Drivetrain::DriveVolts(units::volt_t left, units::volt_t right){
    //set volts and feed drivetrain so no error
    m_leftMotor.SetVoltage(left);
    m_rightMotor.SetVoltage(-right);
    m_drive.Feed();
}

frc::DifferentialDriveWheelSpeeds Drivetrain::GetWheelSpeeds(){
    return {units::meters_per_second_t(m_leftEncoder.GetRate()),
            units::meters_per_second_t(m_rightEncoder.GetRate())};
}

frc::Pose2d Drivetrain::GetPose(){
    return m_odometry.GetPose();
}

units::meter_t Drivetrain::GetLeftDistance(){
    return units::meter_t(m_leftEncoder.GetDistance());
}

units::meter_t Drivetrain::GetRightDistance(){
    return units::meter_t(m_rightEncoder.GetDistance());
}

units::meter_t Drivetrain::GetAverageDistance(){
    return (GetLeftDistance() + GetRightDistance()) / 2.0;
}

bool Drivetrain::IsStopped(){
    return m_leftEncoder.GetStopped() && m_rightEncoder.GetStopped();
}

void Drivetrain::ZeroEncoders(){
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
}
void Drivetrain::ResetOdometry(frc::Pose2d pose){
    ZeroEncoders();
    m_odometry.ResetPosition(pose, frc::Rotation2d(units::degree_t(-m_gyro.GetAngleZ())));
}