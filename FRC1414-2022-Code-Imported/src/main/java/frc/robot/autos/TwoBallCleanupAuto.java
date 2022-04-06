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
          Constants.STARTING_POSITIONS[1],
          List.of(new Translation2d(5.7, 4.7)),
          new Pose2d(5, 5.84, Rotation2d.fromDegrees(20)),
          Constants.TRAJECTORY_CONFIG),
      TrajectoryGenerator.generateTrajectory(
          new Pose2d(5, 5.84, Rotation2d.fromDegrees(20)),
          List.of(),
          new Pose2d(4.8, 3.3, Rotation2d.fromDegrees(-90)),
          Constants.TRAJECTORY_CONFIG),
      TrajectoryGenerator.generateTrajectory(
          new Pose2d(4.8, 3.3, Rotation2d.fromDegrees(-90)),
          List.of(new Translation2d(6.8, 5.8)),
          new Pose2d(6.2, 7.03, Rotation2d.fromDegrees(120)),
          Constants.TRAJECTORY_CONFIG),
      TrajectoryGenerator.generateTrajectory(
          new Pose2d(6.2, 7.03, Rotation2d.fromDegrees(120)),
          List.of(),
          new Pose2d(5, 6.8, Rotation2d.fromDegrees(180)),
          Constants.TRAJECTORY_CONFIG)
  };

  private SequentialCommandGroup auto;

  public TwoBallCleanupAuto() {
    auto = new SequentialCommandGroup(
        new InstantCommand(
            () -> DrivetrainSubsystem.getInstance().setStartingPosition(Constants.STARTING_POSITIONS[1])),
        new SetTurretPosition(-6700).withTimeout(0.5),
        new FollowTrajectory(trajectories[0]).deadlineWith(new IntakeAndHold()),
        new ParallelCommandGroup(
            new AlignTurret(),
            new Shoot(),
            new IntakeWithoutIndexer()).withTimeout(3),
        new FollowTrajectory(trajectories[1]).deadlineWith(new IntakeAndHold()),
        new FollowTrajectory(trajectories[2]).deadlineWith(new IntakeAndHold()),
        new FollowTrajectory(trajectories[3]).deadlineWith(new IntakeAndHold()),
        new Deload());
  }

  public SequentialCommandGroup getAuto() {
    return auto;
  }
}
