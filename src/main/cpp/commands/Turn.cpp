#include "commands/Turn.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

Turn::Turn(Drivetrain* drivetrain, double degreesToTurn) : 
m_drivetrain(drivetrain),
m_targetDegrees(degreesToTurn) {
    AddRequirements({drivetrain});
}

void Turn::Initialize(){
    std::cout << "Started turning " << m_targetDegrees << " degrees" << std::endl;

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

    if(!frc::SmartDashboard::ContainsKey("Turn FF")){
        frc::SmartDashboard::PutNumber("Turn FF", 0.0);
        frc::SmartDashboard::SetPersistent("Turn FF");
    }


}

void Turn::Execute(){
    m_pid.SetP(frc::SmartDashboard::GetNumber("Turn P", 0.0));
    m_pid.SetI(frc::SmartDashboard::GetNumber("Turn I", 0.0));
    m_pid.SetD(frc::SmartDashboard::GetNumber("Turn D", 0.0));

    //P = 0.0055
    //I = 0.0
    //D = 0.000410
    //FF = 0.25

    double angleZ = m_drivetrain->GetGyro()->GetAngleZ();
    double delta = m_targetDegrees - angleZ;
    m_drivetrain->ArcadeDrive(0.0, m_pid.Calculate(angleZ) + (delta / abs(delta)) * frc::SmartDashboard::GetNumber("Turn FF", 0.0));
}

void Turn::End(bool interrupted){
    std::cout << "Finished turning" << std::endl;

    m_drivetrain->ArcadeDrive(0.0, 0.0);
}

bool Turn::IsFinished(){
    return abs(m_drivetrain->GetGyro()->GetAngleZ() - m_targetDegrees) <= 3.0 && m_drivetrain->IsStopped();
}