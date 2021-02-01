#pragma once

#include "subsystems/Drivetrain.h"
#include "Constants.h"

#include <frc/Joystick.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class DriveTeleop : public frc2::CommandHelper<frc2::CommandBase, DriveTeleop>{
public:
    DriveTeleop(Drivetrain* drivetrain);

    void Execute() override;

private:
    frc::Joystick m_driveJoystick{DRIVER_JOYSTICK};
    Drivetrain* m_drivetrain;
};