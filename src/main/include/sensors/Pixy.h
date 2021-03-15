#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>
#include <frc/I2C.h>

class Pixy {
    public:
    Pixy();
    //Gets device Address
    bool AddresOnly();
    // Confirms Sensor 
    bool VerifySensor();
    //Reads and writes data
    bool Transaction();
};