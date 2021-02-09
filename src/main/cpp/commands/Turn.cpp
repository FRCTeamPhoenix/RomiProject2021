#include "commands/Turn.h"

#include <frc/smartdashboard/SmartDashboard.h>

Turn::Turn(Drivetrain* drivetrain, double degreesToTurn) : 
m_drivetrain(drivetrain),
m_targetDegrees(degreesToTurn) {
    AddRequirements({drivetrain});
}

void Turn::Initialize(){
    m_drivetrain->GetGyro()->Reset();
    m_drivetrain->ArcadeDrive(0.0, 0.0);
    m_pid.SetSetpoint(m_targetDegrees);

    //creates PID entries
    if(!frc::SmartDashboard::ContainsKey("Turn P")){
        frc::SmartDashboard::PutNumber("Turn P", 0.0);
        frc::SmartDashboard::PutNumber("Turn I", 0.0);
        frc::SmartDashboard::PutNumber("Turn D", 0.0);

        frc::SmartDashboard::SetPersistent("Turn P");
        frc::SmartDashboard::SetPersistent("Turn I");
        frc::SmartDashboard::SetPersistent("Turn D");
    }
}

void Turn::Execute(){
    m_pid.SetP(frc::SmartDashboard::GetNumber("Turn P", 0.0));
    m_pid.SetI(frc::SmartDashboard::GetNumber("Turn I", 0.0));
    m_pid.SetD(frc::SmartDashboard::GetNumber("Turn D", 0.0));

    //kc = 0.023
    //pc = 0.32
    //kp = 0.0138
    //ki = 0.08625
    //kd = 0.000552

    m_drivetrain->ArcadeDrive(0.0, -m_pid.Calculate(m_drivetrain->GetGyro()->GetAngleZ()));
}

void Turn::End(bool interrupted){
    m_drivetrain->ArcadeDrive(0.0, 0.0);
}

bool Turn::IsFinished(){
    return false;
}