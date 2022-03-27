package frc.robot.autos;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import java.util.List;

public class FourBallAuto implements AutoInterface {
  private PIDController xController = new PIDController(Constants.DRIVETRAIN_PATH_X_kP, 0, 0);
  private PIDController yController = new PIDController(Constants.DRIVETRAIN_PATH_Y_kP, 0, 0);

  private ProfiledPIDController thetaController = new ProfiledPIDController(
    Constants.DRIVETRAIN_PATH_THETA_kP, 0, 0, Constants.THETA_CONTROLLER_CONSTRAINTS);

  private Trajectory[] trajectories = {
      TrajectoryGenerator.generateTrajectory(
          Constants.STARTING_POSITIONS[0],
          List.of(),
          new Pose2d(7.64, 1.2, Rotation2d.fromDegrees(-90)),
          Constants.TRAJECTORY_CONFIG),
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(7.64, 1.2, Rotation2d.fromDegrees(-90)),
        List.of(),
          new Pose2d(1.43, 1.40, Rotation2d.fromDegrees(45)),
          Constants.TRAJECTORY_CONFIG),
  };

  private SequentialCommandGroup auto;

  public FourBallAuto(
      DrivetrainSubsystem drivetrainSubsystem,
      IntakeSubsystem intakeSubsystem,
      IndexerSubsystem indexerSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      HoodSubsystem hoodSubsystem) {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    auto = new SequentialCommandGroup(
      new ParallelCommandGroup(
        new Intake(indexerSubsystem, intakeSubsystem).withTimeout(2.5),
        new FollowTrajectory(drivetrainSubsystem, trajectories[0]).withTimeout(2.5)
      ),
      new TurnToAngle(drivetrainSubsystem, ()->0 , ()->0, 180).withTimeout(1),
      new ParallelCommandGroup(
        new Shoot(shooterSubsystem, indexerSubsystem)
      ).withTimeout(3),
      new ParallelCommandGroup(
        new Intake(indexerSubsystem, intakeSubsystem).withTimeout(3.5),
        new FollowTrajectory(drivetrainSubsystem, trajectories[1]).withTimeout(2.5)
      ),
      new TurnToAngle(drivetrainSubsystem, ()->0 , ()->0, 25).withTimeout(1),
      new Intake(indexerSubsystem, intakeSubsystem).withTimeout(2),
      new Shoot(shooterSubsystem, indexerSubsystem).withTimeout(2)
    );
  }

  public SequentialCommandGroup getAuto() {
    return auto;
  }
}
