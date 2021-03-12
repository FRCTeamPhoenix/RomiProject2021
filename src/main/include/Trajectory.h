#pragma once

#include <frc2/command/RamseteCommand.h>

#include "subsystems/Drivetrain.h"

class Trajectory{
public:
    static frc2::RamseteCommand GenerateRamseteCommand(Drivetrain* drive);
};