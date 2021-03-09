#include "commands/DriveDistanceGyro.h"
#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

DriveDistanceGyro::DriveDistanceGyro(Drivetrain* drivetrain, units::meter_t distanceToTravel) : 
m_drivetrain(drivetrain),
m_distanceToGo(distanceToTravel){
    AddRequirements({drivetrain});
}

void DriveDistanceGyro::Initialize() {
    std::cout << "Started driving " << m_distanceToGo.to<double>() << " meters"<<std::endl;
    m_drivetrain->GetGyro()->Reset();
    m_drivetrain->ZeroEncoders();
    m_drivetrain->ArcadeDrive(0.0, 0.0);
    m_pid.SetSetpoint(m_distanceToGo.to<double>());
    m_gyroPid.SetSetpoint(0);

    if(!frc::SmartDashboard::ContainsKey("Drive P")){
        frc::SmartDashboard::PutNumber("Drive P", 0.0);
        frc::SmartDashboard::PutNumber("Drive I", 0.0);
        frc::SmartDashboard::PutNumber("Drive D", 0.0);
        frc::SmartDashboard::PutNumber("Gyro P", 0.0);
        frc::SmartDashboard::PutNumber("Gyro I", 0.0);
        frc::SmartDashboard::PutNumber("Gyro D", 0.0);

        frc::SmartDashboard::SetPersistent("Drive P");
        frc::SmartDashboard::SetPersistent("Drive I");
        frc::SmartDashboard::SetPersistent("Drive D");
        frc::SmartDashboard::SetPersistent("Gyro P");
        frc::SmartDashboard::SetPersistent("Gyro I");
        frc::SmartDashboard::SetPersistent("Gyro D");
    }

    if(!frc::SmartDashboard::ContainsKey("Drive FF")){
        frc::SmartDashboard::PutNumber("Drive FF", 0.0);
        frc::SmartDashboard::SetPersistent("Drive FF");
    }
}

void DriveDistanceGyro::Execute(){
    //m_drivetrain->ArcadeDrive(DRIVE_SPEED, 0.0);
    m_pid.SetP(frc::SmartDashboard::GetNumber("Drive P", 0.0));
    m_pid.SetI(frc::SmartDashboard::GetNumber("Drive I", 0.0));
    m_pid.SetD(frc::SmartDashboard::GetNumber("Drive D", 0.0));

    m_gyroPid.SetP(frc::SmartDashboard::GetNumber("Gyro P", 0.0));
    m_gyroPid.SetI(frc::SmartDashboard::GetNumber("Gyro I", 0.0));
    m_gyroPid.SetD(frc::SmartDashboard::GetNumber("Gyro D", 0.0));

    //frc::SmartDashboard::UpdateValues();

    //basic
    //p = 6.0
    //i = 0.0
    //d = 0.0
    //ff = 0.01

    std::cout << m_drivetrain->GetGyro()->GetAngleZ() << std::endl;
    double delta = m_distanceToGo.to<double>() - m_drivetrain->GetAverageDistance().to<double>();
    m_drivetrain->ArcadeDrive(m_pid.Calculate(m_drivetrain->GetAverageDistance().to<double>()) + (delta / abs(delta)) * frc::SmartDashboard::GetNumber("Drive FF", 0.0), m_gyroPid.Calculate(m_drivetrain->GetGyro()->GetAngleZ()));
}

void DriveDistanceGyro::End(bool interrupted){
    std::cout << "Finished driving at " << m_drivetrain->GetAverageDistance() - m_distanceToGo << " meters and " << m_drivetrain->GetGyro()->GetAngleZ() << " degrees" << std::endl;
    m_drivetrain->ArcadeDrive(0.0, 0.0);   
}

bool DriveDistanceGyro::IsFinished() {
    return abs((m_drivetrain->GetAverageDistance() - m_distanceToGo).to<double>()) < 0.0075 && abs(m_drivetrain->GetGyro()->GetAngleZ()) < 10/*&& m_drivetrain->IsStopped()*/;
    std::cout << "finished" << std::endl;
}
