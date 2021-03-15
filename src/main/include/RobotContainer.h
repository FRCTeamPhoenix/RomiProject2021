// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc2/command/Command.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "commands/Trajectory.h"
#include "commands/DriveDistance.h"
#include "commands/Turn.h"
#include "subsystems/Drivetrain.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  void UpdateDebugValues();

 private:
  Drivetrain m_drivetrain;
  frc::SendableChooser<CONTROL_SCHEME> m_teleopScheme;

  frc2::SequentialCommandGroup m_autonomous{
    DriveDistance(&m_drivetrain, 17.8_in),
    Turn(&m_drivetrain, 90.0),
    DriveDistance(&m_drivetrain, 17.6_in),
    Turn(&m_drivetrain, 90.0),
    DriveDistance(&m_drivetrain, 8.8_in),
    Turn(&m_drivetrain, 45.0),
    DriveDistance(&m_drivetrain, 16.1_in),
    Turn(&m_drivetrain, -45.0),
    DriveDistance(&m_drivetrain, 8.8_in),
    Turn(&m_drivetrain, -90.0),
    DriveDistance(&m_drivetrain, 17.6_in),
    Turn(&m_drivetrain, -90.0),
    DriveDistance(&m_drivetrain, 17.8_in)
  };

  void ConfigureButtonBindings();
};
