package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.List;

public class TwoBallCleanupAuto implements AutoInterface {
  private Trajectory[] trajectories = {
      TrajectoryGenerator.generateTrajectory(
          Constants.STARTING_POSITIONS[0],
          List.of(),
          new Pose2d(7.8, 1.25, Rotation2d.fromDegrees(-90)),
          Constants.TRAJECTORY_CONFIG),
      TrajectoryGenerator.generateTrajectory(
          new Pose2d(7.8, 1.25, Rotation2d.fromDegrees(-90)),
          List.of(new Translation2d(6.8, 1.5)),
          new Pose2d(5.25, 2.15, Rotation2d.fromDegrees(85)),
          Constants.TRAJECTORY_CONFIG),
      TrajectoryGenerator.generateTrajectory(
          new Pose2d(5.25, 2.15, Rotation2d.fromDegrees(85)),
          List.of(),
          new Pose2d(2, 2, Rotation2d.fromDegrees(225)),
          Constants.TRAJECTORY_CONFIG),
      TrajectoryGenerator.generateTrajectory(
          new Pose2d(2, 2, Rotation2d.fromDegrees(225)),
          List.of(),
          new Pose2d(4, 4, Rotation2d.fromDegrees(0)),
          Constants.TRAJECTORY_CONFIG)
  };

  private SequentialCommandGroup auto;

  public TwoBallCleanupAuto() {
    auto = new SequentialCommandGroup(
        new InstantCommand(
            () -> DrivetrainSubsystem.getInstance().setStartingPosition(Constants.STARTING_POSITIONS[0])),
        new SetTurretPosition(-6700).withTimeout(0.5),
        new SequentialCommandGroup(
            new FollowTrajectory(trajectories[0]),
            new FollowTrajectory(trajectories[1])).deadlineWith(new IntakeAndHold()),
        new ParallelCommandGroup(
            new AlignTurret(),
            new Shoot(),
            new IntakeWithoutIndexer()).withTimeout(3),
        new SequentialCommandGroup(
            new FollowTrajectory(trajectories[2]),
            new SetTurretPosition(0),
            new WaitCommand(0.5)).deadlineWith(new IntakeAndHold()),
        new FollowTrajectory(trajectories[3]).deadlineWith(new ParallelCommandGroup(
            new AlignTurret(),
            new IntakeAndHold())),
        new ParallelCommandGroup(
            new AlignTurret(),
            new Shoot()));
  }

  public SequentialCommandGroup getAuto() {
    return auto;
  }
}
