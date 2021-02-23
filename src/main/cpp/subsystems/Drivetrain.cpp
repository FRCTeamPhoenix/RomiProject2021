#include "subsystems/Drivetrain.h"

#include <wpi/math>

Drivetrain::Drivetrain(){
    m_leftEncoder.SetDistancePerPulse(wpi::math::pi * WHEEL_DIAMETER.to<double>() / TICKS_PER_REVOLUTION);
    m_rightEncoder.SetDistancePerPulse(wpi::math::pi * WHEEL_DIAMETER.to<double>() / TICKS_PER_REVOLUTION);

    //set the speed thresholds
    m_leftEncoder.SetMinRate(MIN_STOP_SPEED.to<double>());
    m_rightEncoder.SetMinRate(MIN_STOP_SPEED.to<double>());

    m_gyro.Reset();
    ZeroEncoders();
}

void Drivetrain::ArcadeDrive(double moveX, double rotZ){
    m_drive.ArcadeDrive(moveX, rotZ);
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