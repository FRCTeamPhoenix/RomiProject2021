#pragma once

#include "subsystems/Drivetrain.h"
#include "Constants.h"

#include <string>

#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandBase.h>
#include <frc2/Timer.h>
#include <frc2/command/CommandHelper.h>

class DriveTeleop : public frc2::CommandHelper<frc2::CommandBase, DriveTeleop>{
public:
    DriveTeleop(Drivetrain* drivetrain, frc::SendableChooser<CONTROL_SCHEME> *teleopScheme);

    void Initialize() override;

    void Execute() override;

private:
    void RunControlScheme(CONTROL_SCHEME scheme);

    frc::SendableChooser<CONTROL_SCHEME> *m_teleopScheme;
    frc::Joystick m_driveJoystick{DRIVER_JOYSTICK};
    frc2::Timer m_chaosTimer;
    CONTROL_SCHEME m_chaosCurrent = CONTROL_SCHEME::TRIGGERS;
    Drivetrain* m_drivetrain;
};