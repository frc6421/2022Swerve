// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAutoCommand extends SequentialCommandGroup {
  private DrivetrainSubsystem driveSubsystem;

  /** Creates a new testAutoCommand. */
  public TestAutoCommand(DrivetrainSubsystem drive) {
    driveSubsystem = drive;

    addRequirements(drive);

    final TrajectoryConfig forwardConfig = new TrajectoryConfig(
        Constants.MAX_VELOCITY_METERS_PER_SECOND, Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
            .setKinematics(driveSubsystem.m_kinematics);

    final TrajectoryConfig reverseConfig = new TrajectoryConfig(
        Constants.MAX_VELOCITY_METERS_PER_SECOND, Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
            .setKinematics(driveSubsystem.m_kinematics)
            .setReversed(true);

    final Trajectory firstTestTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0))),
        List.of(new Translation2d(0, 1)),
        new Pose2d(3, 1, new Rotation2d(Units.degreesToRadians(90))), forwardConfig);

    final Trajectory secondTestTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(3, 1, new Rotation2d(Units.degreesToRadians(90))),
        List.of(new Translation2d(0, 1)),
        new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0))), reverseConfig);

    var thetaController = new ProfiledPIDController(
        Constants.kP_THETA_CONTROLLER, 0, 0, Constants.THETA_CONTROLLER_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand firstSwerveControllerCommand = new SwerveControllerCommand(
        firstTestTrajectory,
        driveSubsystem::getPose, // Functional interface to feed supplier
        driveSubsystem.m_kinematics,

        // Position controllers
        new PIDController(Constants.P_X_CONTROLLER, 0, 0),
        new PIDController(Constants.P_Y_CONTROLLER, 0, 0),
        thetaController,
        driveSubsystem::autoDrive,
        driveSubsystem);

    SwerveControllerCommand secondSwerveControllerCommand = new SwerveControllerCommand(
        secondTestTrajectory,
        driveSubsystem::getPose, // Functional interface to feed supplier
        driveSubsystem.m_kinematics,

        // Position controllers
        new PIDController(Constants.P_X_CONTROLLER, 0, 0),
        new PIDController(Constants.P_Y_CONTROLLER, 0, 0),
        thetaController,
        driveSubsystem::autoDrive,
        driveSubsystem);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> driveSubsystem.resetOdometry(firstTestTrajectory.getInitialPose())),
      firstSwerveControllerCommand,
      new InstantCommand(() -> driveSubsystem.setDriveChassisSpeed(new ChassisSpeeds(0.0, 0.0, 0.0))),
      secondSwerveControllerCommand,
      new InstantCommand(() -> driveSubsystem.setDriveChassisSpeed(new ChassisSpeeds(0.0, 0.0, 0.0))));
  }
}
