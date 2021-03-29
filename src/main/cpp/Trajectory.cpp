#include "Trajectory.h"

#include "Constants.h"

#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>

frc2::RamseteCommand Trajectory::GenerateRamseteCommand(Drivetrain* drive){
    frc::SimpleMotorFeedforward<units::meters> motorFF(TRAJECTORY::S, TRAJECTORY::V,TRAJECTORY::A);

    auto voltageConstraint = frc::DifferentialDriveVoltageConstraint(
        motorFF,
        TRAJECTORY::DRIVE_KINEMATICS,
        6.5_V
    );
    
    frc::TrajectoryConfig config(TRAJECTORY::MAX_SPEED_METERS_PER_SECOND,
        TRAJECTORY::MAX_ACCEL_METERS_PER_SECOND_SQUARED);

    config.SetKinematics(TRAJECTORY::DRIVE_KINEMATICS);
    config.AddConstraint(voltageConstraint);


    frc::Trajectory firstTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      {frc::Pose2d(7.5_in, 7.5_in, frc::Rotation2d(0_deg)),
      frc::Pose2d(22.5_in, 15_in, frc::Rotation2d(45_deg)),
      frc::Pose2d(30_in, 22.5_in, frc::Rotation2d(0_deg)),
      frc::Pose2d(60_in, 22.5_in, frc::Rotation2d(0_deg)),
      frc::Pose2d(67.5_in, 15_in, frc::Rotation2d(-45_deg)),
      frc::Pose2d(75_in, 7.5_in, frc::Rotation2d(0_deg)),
      frc::Pose2d(82.5_in, 15_in, frc::Rotation2d(90_deg)),
      frc::Pose2d(75_in, 22.5_in, frc::Rotation2d(180_deg)),
      frc::Pose2d(67.5_in, 15_in, frc::Rotation2d(225_deg)),
      frc::Pose2d(60_in, 7.5_in, frc::Rotation2d(180_deg)),
      frc::Pose2d(30_in, 7.5_in, frc::Rotation2d(180_deg)),
      frc::Pose2d(22.5_in, 15_in, frc::Rotation2d(135_deg)),
      frc::Pose2d(15_in, 22.5_in, frc::Rotation2d(180_deg)),
      frc::Pose2d(7.5_in, 22.5_in, frc::Rotation2d(180_deg))},
      config);

    frc2::RamseteCommand ramseteCommand(
        firstTrajectory, [drive](){return drive->GetPose();},
        frc::RamseteController(TRAJECTORY::RAMSETE_B, TRAJECTORY::RAMSETE_ZETA),
        motorFF, TRAJECTORY::DRIVE_KINEMATICS,
        [drive] { return drive->GetWheelSpeeds();},
        frc2::PIDController(TRAJECTORY::TRAJECTORY_P, 0.0, 0.0),
        frc2::PIDController(TRAJECTORY::TRAJECTORY_P, 0.0, 0.0),
        [drive](auto left, auto right) { drive->DriveVolts(left, right); },
        {drive}
    );

    drive->ResetOdometry(firstTrajectory.InitialPose());

    return std::move(ramseteCommand);
}