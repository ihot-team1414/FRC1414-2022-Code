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

public class TwoBallAuto {
  private SequentialCommandGroup auto;

  public TwoBallAuto(DrivetrainSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem, HoodSubsystem hoodSubsystem) {

    

    auto = new  SequentialCommandGroup(
        new IntakeAutoDeployCommand(intakeSubsystem).withTimeout(0.5),
        new ParallelCommandGroup(
          new IntakeAutoCommand(intakeSubsystem).withTimeout(3.5),
          new DriveStraightCommand(drivetrainSubsystem).withTimeout(3.5)
        ),
        new TurnToAngleCommand(drivetrainSubsystem, 180).withTimeout(1),
        new ParallelCommandGroup(
          new InstantCommand(() -> intakeSubsystem.setReverse(), intakeSubsystem),
          new TurretAutoCommand(turretSubsystem),
          new HoodAutoCommand(hoodSubsystem),
          new ShooterCommand(shooterSubsystem, indexerSubsystem)
        ).withTimeout(6)
    );
  }

  public SequentialCommandGroup getAuto() {
    return auto;
  }
}
