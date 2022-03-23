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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

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

  public FourBallAuto(DrivetrainSubsystem drivetrainSubsystem) {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand outside4BallAutoCommands[] = {
      new SwerveControllerCommand(
        outside4BallAutoTrajectories[0],
        () -> m_drivetrain.getPose(),
        Constants.KINEMATICS,
        xController,
        yController,
        thetaController,
        m_drivetrain::setModuleStates,
        m_drivetrain),
      new SwerveControllerCommand(
        outside4BallAutoTrajectories[1],
        () -> m_drivetrain.getPose(),
        Constants.KINEMATICS,
        xController,
        yController,
        thetaController,
        m_drivetrain::setModuleStates,
        m_drivetrain),
    };

    auto = new SequentialCommandGroup(
      new IntakeAutoDeployCommand(m_intakesubsystem).withTimeout(0.5),
      new ParallelCommandGroup(
        new IntakeAutoCommand(m_intakesubsystem).withTimeout(2.5),
        outside4BallAutoCommands[0].withTimeout(2.5)
      ),
      new TurnToAngleCommand(m_drivetrain, 180).withTimeout(1),
      new ParallelCommandGroup(
        new InstantCommand(() -> m_intakesubsystem.setReverse(), m_intakesubsystem),
        new TurretAutoCommand(m_turretSubsystem),
        new HoodAutoCommand(m_hoodSubsystem),
        new ShooterAutoCommand(m_shooterSubsystem),
        new SequentialCommandGroup(
          new WaitCommand(2),
          new IndexerAutoCommand(m_indexersubsystem, () -> true)
        )
      ).withTimeout(4),
      new IntakeAutoDeployCommand(m_intakesubsystem).withTimeout(0.5),
      new ParallelCommandGroup(
        new IntakeAutoCommand(m_intakesubsystem).withTimeout(3.5),
        outside4BallAutoCommands[1].withTimeout(3.5)
      ),
      new TurnToAngleCommand(m_drivetrain, 180).withTimeout(1),
      new ParallelCommandGroup(
        new IntakeAutoCommand(m_intakesubsystem),
        new TurretAutoCommand(m_turretSubsystem),
        new HoodAutoCommand(m_hoodSubsystem),
        new ShooterAutoCommand(m_shooterSubsystem),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new IndexerAutoCommand(m_indexersubsystem, () -> true)
        )
      ).withTimeout(3)
    )
    );
  }

  public SequentialCommandGroup getAuto() {
    return auto;
  }
}
