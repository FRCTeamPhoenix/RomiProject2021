#include "subsystems/Drivetrain.h"

#include <wpi/math>

Drivetrain::Drivetrain(){
    m_leftEncoder.SetDistancePerPulse(wpi::math::pi * WHEEL_DIAMETER.to<double>() / TICKS_PER_REVOLUTION);
    m_rightEncoder.SetDistancePerPulse(wpi::math::pi * WHEEL_DIAMETER.to<double>() / TICKS_PER_REVOLUTION);

    m_gyro.Reset();
    ZeroEncoders();
}

void Drivetrain::ArcadeDrive(double moveX, double rotZ){
    m_drive.ArcadeDrive(moveX, rotZ);
}

void Drivetrain::TankDrive(double left, double right){
    m_drive.TankDrive(left, right);
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

void Drivetrain::ZeroEncoders(){
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
}