/*#include "commands/Trajectory.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory>
#include <iostream>

Trajectory::Trajectory(Drivetrain* drivetrain, units::meter_t midX, units::meter_t midY, units::meter_t endX, units::meter_t endY, units::degree endRot) : 
m_drivetrain(drivetrain),
m_midX(midX),
m_midY(midY),
m_endX(endX),
m_endY(endY),
m_endRot(endRot) {
    AddRequirements({drivetrain});
}

void Trajectory::Initialize(){
    std::cout << "Started driving to " << m_endX << ", " << m_endY << std::endl;

    m_drivetrain->GetGyro()->Reset();
    m_drivetrain->ArcadeDrive(0.0, 0.0);
    //m_pid.SetSetpoint(m_endRot.to<double>());
    //m_pid2.SetSetpoint(m_endX.to<double>());

    // Create a voltage constraint to ensure we don't accelerate too fast
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
      frc::SimpleMotorFeedforward<units::meters>(
          DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
      DriveConstants::kDriveKinematics, 10_V);

  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(DriveConstants::kDriveKinematics);
  // Apply the voltage constraint
  config.AddConstraint(autoVoltageConstraint);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass the config
      config);

  frc2::RamseteCommand ramseteCommand(
      exampleTrajectory, [this]() { return m_drive.GetPose(); },
      frc::RamseteController(AutoConstants::kRamseteB,
                             AutoConstants::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(
          DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
      DriveConstants::kDriveKinematics,
      [this] { return m_drive.GetWheelSpeeds(); },
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      [this](auto left, auto right) { m_drive.TankDriveVolts(left, right); },
      {&m_drive});

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());

  // no auto
  return new frc2::SequentialCommandGroup(
      std::move(ramseteCommand),
      frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));
}

void Trajectory::Execute(){
    m_pid.SetP(frc::SmartDashboard::GetNumber("Turn P", 0.0));
    m_pid.SetI(frc::SmartDashboard::GetNumber("Turn I", 0.0));
    m_pid.SetD(frc::SmartDashboard::GetNumber("Turn D", 0.0));

    //P = 0.0055
    //I = 0.0
    //D = 0.000410
    //FF = 0.25

    double angleZ = m_drivetrain->GetGyro()->GetAngleZ();
    m_drivetrain->ArcadeDrive(0.0, -m_pid.Calculate(angleZ) - (angleZ / abs(angleZ)) * frc::SmartDashboard::GetNumber("Turn FF", 0.0));
}

void Trajectory::End(bool interrupted){
    std::cout << "Finished turning" << std::endl;

    m_drivetrain->ArcadeDrive(0.0, 0.0);
}

bool Trajectory::IsFinished(){
    //return m_drivetrain->GetAverageDistance() >= m_distanceToGo;
    //return abs(m_drivetrain->GetGyro()->GetAngleZ() - m_targetDegrees) <= 2.0;
}*/