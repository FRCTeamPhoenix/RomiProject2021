#pragma once

#include "subsystems/Drivetrain.h"
#include "Constants.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class DriveDistance: public frc2::CommandHelper<frc2::CommandBase, DriveDistance>{
public:
    DriveDistance(Drivetrain* drivetrain, units::meter_t distanceToTravel);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    Drivetrain* m_drivetrain;
    units::meter_t m_distanceToGo;
};