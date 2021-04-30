#include "Trajectory.h"

#include "Constants.h"

#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc2/command/InstantCommand.h>

frc2::SequentialCommandGroup Trajectory::GenerateRamseteCommand(Drivetrain* drive, std::vector<frc::Pose2d> poses){
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
      poses,
      config);

    frc2::RamseteCommand ramseteCommand(
        firstTrajectory, [drive](){return drive->GetPose();},
        frc::RamseteController(TRAJECTORY::RAMSETE_B, TRAJECTORY::RAMSETE_ZETA),
        motorFF, TRAJECTORY::DRIVE_KINEMATICS,
        [drive] { return drive->GetWheelSpeeds();},
        frc2::PIDController(TRAJECTORY::TRAJECTORY_P - 0.1, 0.0, 0.0),
        frc2::PIDController(TRAJECTORY::TRAJECTORY_P, 0.0, 0.0),
        [drive](auto left, auto right) { drive->DriveVolts(left, right); },
        {drive}
    );

    return std::move(frc2::SequentialCommandGroup{
        frc2::InstantCommand([drive, firstTrajectory]{drive->ResetOdometry(firstTrajectory.InitialPose());}),
        std::move(ramseteCommand)
    });
}