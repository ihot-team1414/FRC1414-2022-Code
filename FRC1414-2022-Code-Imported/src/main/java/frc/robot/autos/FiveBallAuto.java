package frc.robot.autos;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.*;
import java.util.List;

public class FiveBallAuto implements AutoInterface {
  private ProfiledPIDController thetaController = new ProfiledPIDController(
    Constants.DRIVETRAIN_PATH_THETA_kP, 0, 0, Constants.THETA_CONTROLLER_CONSTRAINTS
  );

  private Trajectory[] trajectories = {
    TrajectoryGenerator.generateTrajectory(
      Constants.STARTING_POSITIONS[0],
      List.of(),
      new Pose2d(7.9, 1, Rotation2d.fromDegrees(-90)),
      Constants.TRAJECTORY_CONFIG),
    TrajectoryGenerator.generateTrajectory(
      new Pose2d(7.9, 1, Rotation2d.fromDegrees(-90)),
      List.of(new Translation2d(6, 1.02)),
      new Pose2d(5, 1.5, Rotation2d.fromDegrees(85)),
      Constants.TRAJECTORY_CONFIG),
    TrajectoryGenerator.generateTrajectory(
      new Pose2d(5, 1.5, Rotation2d.fromDegrees(85)),
      List.of(),
      new Pose2d(1.3, 1.2, Rotation2d.fromDegrees(225)),
      Constants.TRAJECTORY_CONFIG),
    TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.3, 1.2, Rotation2d.fromDegrees(225)),
      List.of(),
      new Pose2d(3, 3, Rotation2d.fromDegrees(0)),
      Constants.TRAJECTORY_CONFIG)
  };

  private SequentialCommandGroup auto;

  public FiveBallAuto() {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    auto = new SequentialCommandGroup(
      new SequentialCommandGroup(
        new FollowTrajectory(trajectories[0]),
        new FollowTrajectory(trajectories[1])
      ).deadlineWith(new IntakeAndHold()),
      new ParallelCommandGroup(
          new Shoot(),
          new IntakeWithoutIndexer()
      ).withTimeout(4),
      new SequentialCommandGroup(
        new FollowTrajectory(trajectories[2]),
        new WaitCommand(0.5)
      ).deadlineWith(new IntakeAndHold()),
      new FollowTrajectory(trajectories[3]),
      new Shoot()
    );
  }

  public SequentialCommandGroup getAuto() {
    return auto;
  }
}
