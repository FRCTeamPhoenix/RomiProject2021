#pragma once

#include <frc2/command/RamseteCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/Drivetrain.h"

class Trajectory{
public:
    static frc2::SequentialCommandGroup GenerateRamseteCommand(Drivetrain* drive, std::vector<frc::Pose2d> poses);
};