#include "commands/DriveTeleop.h"

#include <algorithm>
#include <iostream>

DriveTeleop::DriveTeleop(Drivetrain* drivetrain,
 frc::SendableChooser<CONTROL_SCHEME> *teleopScheme) :
 m_drivetrain(drivetrain),
 m_teleopScheme(teleopScheme) {
    AddRequirements({drivetrain});
}

void DriveTeleop::Initialize(){
    m_chaosTimer.Start();
}

void DriveTeleop::Execute(){
    CONTROL_SCHEME mode = m_teleopScheme->GetSelected();
    
    if(mode != CONTROL_SCHEME::CHAOS){
        RunControlScheme(mode);
    }else{
        //prepare the chaos modifier
        if(m_chaosTimer.HasPeriodPassed(CHAOS_INTERVAL)){
            //choose a random message and mode
            int message = rand() % sizeof(CHAOS_MESSAGES) / sizeof(std::string);
            int modeIndex = rand() % sizeof(CHAOS_POSSIBILITIES) / sizeof(CONTROL_SCHEME);
            std::cout << CHAOS_MESSAGES[message] << std::endl;
            //set the new mode
            m_chaosCurrent = CHAOS_POSSIBILITIES[modeIndex];
        }

        //run using the current chaos mode
        RunControlScheme(m_chaosCurrent);
    }
    
}

void DriveTeleop::RunControlScheme(CONTROL_SCHEME scheme){
    switch (scheme)
    {
    case CONTROL_SCHEME::ONE_STICK:
        m_drivetrain->ArcadeDrive(-m_driveJoystick.GetRawAxis(LEFTSTICK_Y), m_driveJoystick.GetRawAxis(LEFTSTICK_X));
        break;
    case CONTROL_SCHEME::TWO_STICK:
        m_drivetrain->ArcadeDrive(-m_driveJoystick.GetRawAxis(LEFTSTICK_Y), m_driveJoystick.GetRawAxis(RIGHTSTICK_X));
        break;
    case CONTROL_SCHEME::TANKDRIVE:
        m_drivetrain->TankDrive(-m_driveJoystick.GetRawAxis(LEFTSTICK_Y), -m_driveJoystick.GetRawAxis(RIGHTSTICK_Y));
        break;
    case CONTROL_SCHEME::QWOP:
    {
        double zButton = m_driveJoystick.GetRawButton(BUTTON_B) - m_driveJoystick.GetRawButton(BUTTON_X);
        zButton = std::max(std::min(zButton, 1.0), -1.0);
        double xButton = m_driveJoystick.GetRawButton(BUTTON_Y) - m_driveJoystick.GetRawButton(BUTTON_A);
        xButton = std::max(std::min(xButton, 1.0), -1.0);
        m_drivetrain->ArcadeDrive(xButton, zButton);
        break;
    }
    case CONTROL_SCHEME::TRIGGERS:
    default:
        double zRot = m_driveJoystick.GetRawAxis(RIGHT_TRIGGER) - m_driveJoystick.GetRawAxis(LEFT_TRIGGER);
        zRot = std::max(std::min(zRot, 1.0), -1.0);
        m_drivetrain->ArcadeDrive(-m_driveJoystick.GetRawAxis(LEFTSTICK_Y), zRot);
        break;
    }
}