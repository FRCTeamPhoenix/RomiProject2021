#include "commands/DriveDistance.h"

DriveDistance::DriveDistance(Drivetrain* drivetrain, units::meter_t distanceToTravel) : 
m_drivetrain(drivetrain),
m_distanceToGo(distanceToTravel){
    AddRequirements({drivetrain});
}

void DriveDistance::Initialize() {
    m_drivetrain->ArcadeDrive(0.0, 0.0);
    m_drivetrain->ZeroEncoders();
}

void DriveDistance::Execute(){
    m_drivetrain->ArcadeDrive(DRIVE_SPEED, 0.0);
}

void DriveDistance::End(bool interrupted){
    m_drivetrain->ArcadeDrive(0.0, 0.0);   
}

bool DriveDistance::IsFinished() {
    return m_drivetrain->GetAverageDistance() > m_distanceToGo;
}
