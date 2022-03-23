package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class FourBallAuto {
  private PIDController xController = new PIDController(Constants.DRIVETRAIN_PATH_X_kP, 0, 0);
  private PIDController yController = new PIDController(Constants.DRIVETRAIN_PATH_Y_kP, 0, 0);

  private ProfiledPIDController thetaController = new ProfiledPIDController(
    Constants.DRIVETRAIN_PATH_THETA_kP, 0, 0, Constants.THETA_CONTROLLER_CONSTRAINTS);

  private Trajectory outside4BallAutoTrajectories[] = {
    TrajectoryGenerator.generateTrajectory(
      Constants.STARTING_POSITIONS[0],
      List.of(),
      new Pose2d(7.64, 0.72, Rotation2d.fromDegrees(90)),
      Constants.TRAJECTORY_CONFIG),
    TrajectoryGenerator.generateTrajectory(
      new Pose2d(7.64, 0.72, Rotation2d.fromDegrees(270)),
      List.of(),
      new Pose2d(1.43, 1.40, Rotation2d.fromDegrees(45)),
      Constants.TRAJECTORY_CONFIG),
  };

  private SequentialCommandGroup auto;

  public FourBallAuto(DrivetrainSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem, HoodSubsystem hoodSubsystem) {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand outside4BallAutoCommands[] = {
      new SwerveControllerCommand(
        outside4BallAutoTrajectories[0],
        () -> drivetrainSubsystem.getPose(),
        Constants.KINEMATICS,
        xController,
        yController,
        thetaController,
        drivetrainSubsystem::setModuleStates,
        drivetrainSubsystem),
      new SwerveControllerCommand(
        outside4BallAutoTrajectories[1],
        () -> drivetrainSubsystem.getPose(),
        Constants.KINEMATICS,
        xController,
        yController,
        thetaController,
        drivetrainSubsystem::setModuleStates,
        drivetrainSubsystem),
    };

    auto = new SequentialCommandGroup(
      new IntakeAutoDeployCommand(intakeSubsystem).withTimeout(0.5),
      new ParallelCommandGroup(
        new IntakeAutoCommand(intakeSubsystem).withTimeout(2.5),
        outside4BallAutoCommands[0].withTimeout(2.5)
      ),
      new TurnToAngleCommand(drivetrainSubsystem, 180).withTimeout(1),
      new ParallelCommandGroup(
        new InstantCommand(() -> intakeSubsystem.setReverse(), intakeSubsystem),
        new TurretAutoCommand(turretSubsystem),
        new HoodAutoCommand(hoodSubsystem),
        new ShooterCommand(shooterSubsystem, indexerSubsystem)
      ).withTimeout(3),
      new IntakeAutoDeployCommand(intakeSubsystem).withTimeout(0.5),
      new ParallelCommandGroup(
        new IntakeAutoCommand(intakeSubsystem).withTimeout(3.5),
        outside4BallAutoCommands[1].withTimeout(3.5)
      ),
      new TurnToAngleCommand(drivetrainSubsystem, 25).withTimeout(1),
      new ParallelCommandGroup(
        new IntakeAutoCommand(intakeSubsystem),
        new TurretAutoCommand(turretSubsystem),
        new HoodAutoCommand(hoodSubsystem),
        new ShooterCommand(shooterSubsystem, indexerSubsystem)
      ).withTimeout(3)
    );
  }

  public SequentialCommandGroup getAuto() {
    return auto;
  }
}
