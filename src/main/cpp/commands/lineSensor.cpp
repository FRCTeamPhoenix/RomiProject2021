//i2c_Lidar = new I2C(I2C::Port::kOnboard, LIDAR_I2C_DEFAULT_ADDR);
//i2c_Lidar->Write(0x00, 0x00);
//i2c_Lidar->ReadOnly(1,LidarMassData);
//(0b00001 & 0b00001)

#include "commands/lineSensor.h"
#include <math.h>
#include "Constants.h"
#include <iostream>

lineSensor::lineSensor(Drivetrain* drivetrain) : 
m_drivetrain(drivetrain){
    AddRequirements({drivetrain});
}

void lineSensor::Initialize() {
    std::cout << "Started following line" << std::endl;
    m_drivetrain->ArcadeDrive(0.0, 0.0);
    m_drivetrain->ZeroEncoders();
}

void lineSensor::Execute(){
    //m_drivetrain->ArcadeDrive(DRIVE_SPEED, 0.0);

    //P = 0.0055
    //I = 0.0
    //D = 0.000410
    //FF = 0.25

    //double angleZ = m_drivetrain->GetGyro()->GetAngleZ();
    uint8_t sensorData;
    m_i2c_Line.Read(0x44, 8, &sensorData);

    //int sensorDataList[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    int sensorDataValue = 0;
	int sensorDataValue2 = 0;
	//std::cout << ((sensorData & 0b00000001));
	if ((sensorData & 0b00000001) == 0b000000001) {
		//sensorDataList[0] = 1;
		sensorDataValue = 4;
	}
	if ((sensorData & 0b00000010) == 0b00000010) {
		//sensorDataList[1] = 1;
		if (sensorDataValue != 0) {
			sensorDataValue2 = 3;
		}
		else {
			sensorDataValue = 3;
		}
	}
	if ((sensorData & 0b00000100) == 0b00000100) {
		//sensorDataList[2] = 1;
		if (sensorDataValue != 0) {
			sensorDataValue2 = 2;
		}
		else {
			sensorDataValue = 2;
		}
	}
	if ((sensorData & 0b00001000) == 0b00001000) {
		//sensorDataList[3] = 1;
		if (sensorDataValue != 0) {
			sensorDataValue2 = 1;
		}
		else {
			sensorDataValue = 1;
		}
	}
	if ((sensorData & 0b00010000) == 0b00010000) {
		//sensorDataList[4] = 1;
		if (sensorDataValue != 0) {
			sensorDataValue2 = -1;
		}
		else {
			sensorDataValue = -1;
		}
	}
	if ((sensorData & 0b00100000) == 0b00100000) {
		//sensorDataList[5] = 1;
		if (sensorDataValue != 0) {
			sensorDataValue2 = -2;
		}
		else {
			sensorDataValue = -2;
		}
	}
	if ((sensorData & 0b01000000) == 0b01000000) {
		//sensorDataList[6] = 1;
		if (sensorDataValue != 0) {
			sensorDataValue2 = -3;
		}
		else {
			sensorDataValue = -3;
		}
	}
	if ((sensorData & 0b10000000) == 0b10000000) {
		//sensorDataList[7] = 1;
		if (sensorDataValue != 0) {
			sensorDataValue2 = -4;
			//std::cout << "hhhh";
		}
		else {
			//std::cout << sensorDataValue << " == 0" << std::endl;
			sensorDataValue = -4;
		}
	}

    double angleZ = sensorDataValue;
    if (sensorDataValue2 != 0) {
        angleZ = ((sensorDataValue + sensorDataValue2)/2);
    }
    m_drivetrain->ArcadeDrive(DRIVE_SPEED, -m_pid.Calculate(angleZ) - (angleZ / abs(angleZ)));
}

void lineSensor::End(bool interrupted){
    std::cout << "Finished following line" << std::endl;
    m_drivetrain->ArcadeDrive(0.0, 0.0);   
}

bool lineSensor::IsFinished() {
    return false;
}
