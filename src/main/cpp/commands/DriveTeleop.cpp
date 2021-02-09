#include "commands/DriveTeleop.h"

#include <algorithm>

DriveTeleop::DriveTeleop(Drivetrain* drivetrain) : m_drivetrain(drivetrain){
    AddRequirements({drivetrain});
}

void DriveTeleop::Execute(){
    double zRot = m_driveJoystick.GetRawAxis(TURN_RIGHT_AXIS) - m_driveJoystick.GetRawAxis(TURN_LEFT_AXIS);
    zRot = std::max(std::min(zRot, 1.0), -1.0);
    m_drivetrain->ArcadeDrive(m_driveJoystick.GetRawAxis(MOVE_X_AXIS), zRot);
}