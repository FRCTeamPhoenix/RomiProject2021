#pragma once

#include "subsystems/Drivetrain.h"
#include "Constants.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>

class Turn : public frc2::CommandHelper<frc2::CommandBase, Turn>{
public:
    Turn(Drivetrain* drivetrain, double degreesToTurn);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    double m_targetDegrees;
    Drivetrain* m_drivetrain;

    frc2::PIDController m_pid{0.0, 0.0, 0.0};

};