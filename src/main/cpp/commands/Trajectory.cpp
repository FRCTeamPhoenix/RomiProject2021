#include "commands/Trajectory.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>


   Trajectory::Trajectory(Drivetrain* drivetrain){
       AddRequirements({drivetrain});
   }

    void Trajectory::Initialize(){
        m_drivetrain->ArcadeDrive(0.0, 0.0);
    }

    void Trajectory::Execute(){
       
    }

    void Trajectory::End(bool interrupted){
        m_drivetrain->ArcadeDrive(0.0, 0.0);
    }

    bool Trajectory::IsFinished(){
            return m_drivetrain->GetAverageDistance() >= m_distanceToGo;
    }
