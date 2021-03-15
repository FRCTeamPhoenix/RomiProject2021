//i2c_Lidar = new I2C(I2C::Port::kOnboard, LIDAR_I2C_DEFAULT_ADDR);
//i2c_Lidar->Write(0x00, 0x00);
//i2c_Lidar->ReadOnly(1,LidarMassData);
//(0b00001 & 0b00001)

// if > 8
//turn right

//if < 8
//turn left

//(input - 4) then can use for PID

#pragma once

//#include <hal/SimDevice.h>
//#include <I2C.h>
#include <frc/I2C.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Drivetrain.h"
#include <frc/controller/PIDController.h>

class lineSensor: public frc2::CommandHelper<frc2::CommandBase, lineSensor>{
 public:
  lineSensor();

  /**
   * Gets the value from the sensor
   */
  //double GetSensorValue();
  lineSensor(Drivetrain* drivetrain);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  /**
   * Resets the gyro
   */
  //void Reset();

  //void UpdateDebugValues();
 private:
  //HAL_I2C_kInvalid
  //HAL_I2C_kMXP
  //HAL_I2C_kOnboard
  //HAL_I2CPort
  //HAL_InitializeI2C
  //HAL_ReadI2C
  //HAL_TransactionI2C
  //HAL_WriteI2C

  //hal::I2CPort 

  frc::I2C m_i2c_Line = frc::I2C::I2C(frc::I2C::Port::kOnboard, 0);

  /*double m_p = 0.0;
  double m_i = 0.0;
  double m_d = 0.0;*/
  //frc2::PIDController m_pid{m_p, m_i, m_d};
  frc2::PIDController m_pid{0.0, 0.0, 0.0};

  Drivetrain* m_drivetrain;
};