#pragma once

#include "subsystems/Drivetrain.h"
#include "Constants.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>

class Trajectory : public frc2::CommandHelper<frc2::CommandBase, Trajectory>{
public:
    Trajectory(Drivetrain* drivetrain, units::meter_t midX, units::meter_t midY, units::meter_t endX, units::meter_t endY, units::degree endRot);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    units::meter_t m_midX, m_midY, m_endX, m_endY;
    units::degree m_endRot;
    Drivetrain* m_drivetrain;

    frc2::PIDController m_pid{0.0, 0.0, 0.0};
    frc2::PIDController m_pid2{0.0, 0.0, 0.0};
};