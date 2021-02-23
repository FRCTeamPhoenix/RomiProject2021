#include "commands/DriveDistance.h"
#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

DriveDistance::DriveDistance(Drivetrain* drivetrain, units::meter_t distanceToTravel) : 
m_drivetrain(drivetrain),
m_distanceToGo(distanceToTravel){
    AddRequirements({drivetrain});
}

void DriveDistance::Initialize() {
    std::cout << "Started driving " << m_distanceToGo.to<double>() << " meters"<<std::endl;
    m_drivetrain->ZeroEncoders();
    m_drivetrain->ArcadeDrive(0.0, 0.0);
    m_pid.SetSetpoint(m_distanceToGo.to<double>());

    if(!frc::SmartDashboard::ContainsKey("Drive P")){
        frc::SmartDashboard::PutNumber("Drive P", 0.0);
        frc::SmartDashboard::PutNumber("Drive I", 0.0);
        frc::SmartDashboard::PutNumber("Drive D", 0.0);

        frc::SmartDashboard::SetPersistent("Drive P");
        frc::SmartDashboard::SetPersistent("Drive I");
        frc::SmartDashboard::SetPersistent("Drive D");
    }

    if(!frc::SmartDashboard::ContainsKey("Drive FF")){
        frc::SmartDashboard::PutNumber("Drive FF", 0.0);
        frc::SmartDashboard::SetPersistent("Drive FF");
    }
}

void DriveDistance::Execute(){
    //m_drivetrain->ArcadeDrive(DRIVE_SPEED, 0.0);
    m_pid.SetP(frc::SmartDashboard::GetNumber("Drive P", 0.0));
    m_pid.SetI(frc::SmartDashboard::GetNumber("Drive I", 0.0));
    m_pid.SetD(frc::SmartDashboard::GetNumber("Drive D", 0.0));

    //frc::SmartDashboard::UpdateValues();

    //basic
    //p = 6.0
    //i = 0.0
    //d = 0.0
    //ff = 0.01

    m_drivetrain->ArcadeDrive(m_pid.Calculate(m_drivetrain->GetAverageDistance().to<double>()) - frc::SmartDashboard::GetNumber("Drive FF", 0.0), 0.0);
}

void DriveDistance::End(bool interrupted){
    std::cout << "Finished drving at " << m_drivetrain->GetAverageDistance() - m_distanceToGo << " meters" << std::endl;
    m_drivetrain->ArcadeDrive(0.0, 0.0);   
}

bool DriveDistance::IsFinished() {
    return abs((m_drivetrain->GetAverageDistance() - m_distanceToGo).to<double>()) < 0.0075;
}
