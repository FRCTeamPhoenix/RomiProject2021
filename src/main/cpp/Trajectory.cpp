#include "Trajectory.h"

#include "Constants.h"

#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>

frc2::RamseteCommand Trajectory::generateRamseteCommand(){
    auto voltageConstraint = frc::DifferentialDriveVoltageConstraint(
        frc::SimpleMotorFeedforward<units::meters>(TRAJECTORY::VOLTS,
            TRAJECTORY::VOLT_SECONDS_PER_METER,
            TRAJECTORY::VOLT_SECONDS_SQUARED_PER_METER),
        TRAJECTORY::DRIVE_KINEMATICS,
        10_V
    );
    
    frc::TrajectoryConfig config(TRAJECTORY::MAX_SPEED_METERS_PER_SECOND,
        TRAJECTORY::MAX_ACCEL_METERS_PER_SECOND_SQUARED);

    config.SetKinematics(TRAJECTORY::DRIVE_KINEMATICS);
    config.AddConstraint(voltageConstraint);

    frc::Trajectory firstTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        {frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
        frc::Pose2d(0.2286_m * 1.95, 0.2286_m, frc::Rotation2d(90_deg)),
        frc::Pose2d(0.2286_m * 1.5, 0.2286_m * 2, frc::Rotation2d(180_deg)),
        frc::Pose2d(0.2286_m * -0.4125, 0.2286_m + 0.35_m, frc::Rotation2d(180_deg)),
        frc::Pose2d(-0.2286_m + 0.05_m, 0.2286_m * 2.4, frc::Rotation2d(45_deg)),
        frc::Pose2d(0.2286_m * 0.8, 0.2286_m * 2.8, frc::Rotation2d(0_deg))},
        config
    );

    //reset drivetrain odeometry here
}