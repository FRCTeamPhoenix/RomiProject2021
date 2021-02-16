// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include "commands/DriveTeleop.h"

#include <frc/smartdashboard/SmartDashboard.h>

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  m_drivetrain.SetDefaultCommand(DriveTeleop(&m_drivetrain, &m_teleopScheme));

  m_teleopScheme.AddDefault("Trigger Turn", "tmode");
  m_teleopScheme.AddOption("One Stick", "1mode");
  m_teleopScheme.AddOption("Two Sticks", "2mode");
  m_teleopScheme.AddOption("Tank Drive", "tdrive");
  frc::SmartDashboard::PutData(&m_teleopScheme);

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomous;
}

void RobotContainer::UpdateDebugValues(){
  m_drivetrain.GetGyro()->UpdateDebugValues();
}