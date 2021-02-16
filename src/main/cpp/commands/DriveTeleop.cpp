#include "commands/DriveTeleop.h"

#include <algorithm>

DriveTeleop::DriveTeleop(Drivetrain* drivetrain,
 frc::SendableChooser<std::string> *teleopScheme) :
 m_drivetrain(drivetrain),
 m_teleopScheme(teleopScheme) {
    AddRequirements({drivetrain});
}

void DriveTeleop::Execute(){
    std::string mode = m_teleopScheme->GetSelected();
    
    if(mode == "tmode"){
        double zRot = m_driveJoystick.GetRawAxis(RIGHT_TRIGGER) - m_driveJoystick.GetRawAxis(LEFT_TRIGGER);
        zRot = std::max(std::min(zRot, 1.0), -1.0);
        m_drivetrain->ArcadeDrive(-m_driveJoystick.GetRawAxis(LEFTSTICK_Y), zRot);
    }else if(mode == "1mode"){
        m_drivetrain->ArcadeDrive(-m_driveJoystick.GetRawAxis(LEFTSTICK_Y), m_driveJoystick.GetRawAxis(LEFTSTICK_X));
    }else if(mode == "2mode"){
        m_drivetrain->ArcadeDrive(-m_driveJoystick.GetRawAxis(LEFTSTICK_Y), m_driveJoystick.GetRawAxis(RIGHTSTICK_X));
    }else if(mode == "tdrive"){
        m_drivetrain->TankDrive(-m_driveJoystick.GetRawAxis(LEFTSTICK_Y), -m_driveJoystick.GetRawAxis(RIGHTSTICK_Y));
    }
    
}