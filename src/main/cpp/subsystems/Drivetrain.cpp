#include "subsystems/Drivetrain.h"

Drivetrain::Drivetrain(){

}

void Drivetrain::ArcadeDrive(float moveX, float rotZ){
    m_drive.ArcadeDrive(moveX, rotZ);
}