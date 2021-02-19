#pragma once

#include "subsystems/Drivetrain.h"
#include "Constants.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>

class Trajectory : public frc2::CommandHelper<frc2::CommandBase, Trajectory>{
public:
    Trajectory(Drivetrain* drivetrain);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    Drivetrain* m_drivetrain;
    units::meter_t m_distanceToGo;
    frc2::PIDController m_pid{0.0, 0.0, 0.0};

};
