// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include "commands/DriveTeleop.h"
#include "Trajectory.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  m_drivetrain.SetDefaultCommand(DriveTeleop(&m_drivetrain, &m_teleopScheme));

  m_teleopScheme.AddDefault("Trigger Turn", CONTROL_SCHEME::TRIGGERS);
  m_teleopScheme.AddOption("One Stick", CONTROL_SCHEME::ONE_STICK);
  m_teleopScheme.AddOption("Two Sticks", CONTROL_SCHEME::TWO_STICK);
  m_teleopScheme.AddOption("Tank Drive", CONTROL_SCHEME::TANKDRIVE);
  m_teleopScheme.AddOption("Chaos Mode", CONTROL_SCHEME::CHAOS);
  m_teleopScheme.AddOption("QWOP Mode", CONTROL_SCHEME::QWOP);
  frc::SmartDashboard::PutData(&m_teleopScheme);

  // Configure the button bindings
  ConfigureButtonBindings();

  //set the random seed
  srand(time(NULL));
}

RobotContainer::~RobotContainer(){
  if(m_autonomous != nullptr)
    delete m_autonomous;
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  if(m_autonomous != nullptr) delete m_autonomous;

  //follows a course to pick up the three balls
  m_autonomous = new frc2::SequentialCommandGroup(
    Trajectory::GenerateRamseteCommand(&m_drivetrain,
      {frc::Pose2d(7.5_in, 22.5_in, frc::Rotation2d(0_deg)),
      frc::Pose2d(15_in, 22.5_in, frc::Rotation2d(0_deg)),
      frc::Pose2d(22.5_in, 30_in, frc::Rotation2d(90_deg)),
      frc::Pose2d(22.5_in, 37.5_in, frc::Rotation2d(90_deg))}),
    Turn(&m_drivetrain, 180.0),
    Trajectory::GenerateRamseteCommand(&m_drivetrain,
      {frc::Pose2d(22.5_in, 37.5_in, frc::Rotation2d(-90_deg)),
      frc::Pose2d(22.5_in, 30_in, frc::Rotation2d(-90_deg)),
      frc::Pose2d(30_in, 15_in, frc::Rotation2d(-90_deg)),
      frc::Pose2d(37.5_in, 7.5_in, frc::Rotation2d(0_deg)),
      frc::Pose2d(45_in, 15_in, frc::Rotation2d(90_deg)),
      frc::Pose2d(45_in, 37.5_in, frc::Rotation2d(90_deg))}),
    Turn(&m_drivetrain, 180.0),
    Trajectory::GenerateRamseteCommand(&m_drivetrain,
      {frc::Pose2d(45_in, 37.5_in, frc::Rotation2d(-90_deg)),
      frc::Pose2d(45_in, 15_in, frc::Rotation2d(-90_deg)),
      frc::Pose2d(52.5_in, 7.5_in, frc::Rotation2d(0_deg)),
      frc::Pose2d(60_in, 7.5_in, frc::Rotation2d(0_deg)),
      frc::Pose2d(67.5_in, 15_in, frc::Rotation2d(90_deg)),
      frc::Pose2d(67.5_in, 37.5_in, frc::Rotation2d(90_deg))}),
    Turn(&m_drivetrain, 180.0),
    Trajectory::GenerateRamseteCommand(&m_drivetrain,
      {frc::Pose2d(67.5_in, 37.5_in, frc::Rotation2d(-90_deg)),
      frc::Pose2d(67.5_in, 30_in, frc::Rotation2d(-90_deg)),
      frc::Pose2d(75_in, 22.5_in, frc::Rotation2d(0_deg)),
      frc::Pose2d(82.5_in, 22.5_in, frc::Rotation2d(0_deg))}),
    frc2::InstantCommand([this]{m_drivetrain.DriveVolts(0_V, 0_V); })
  );

  return m_autonomous;
}

void RobotContainer::UpdateDebugValues(){
  m_drivetrain.GetGyro()->UpdateDebugValues();
}