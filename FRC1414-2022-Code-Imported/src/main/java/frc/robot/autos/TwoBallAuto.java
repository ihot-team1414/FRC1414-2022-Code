package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class TwoBallAuto implements AutoInterface {
  private SequentialCommandGroup auto;

  public TwoBallAuto(
    DrivetrainSubsystem drivetrainSubsystem,
    IntakeSubsystem intakeSubsystem,
    IndexerSubsystem indexerSubsystem,
    ShooterSubsystem shooterSubsystem,
    TurretSubsystem turretSubsystem,
    HoodSubsystem hoodSubsystem
  ) {

    auto = new SequentialCommandGroup(
        new ParallelCommandGroup(
          new Intake(indexerSubsystem, intakeSubsystem).withTimeout(3.5),
          new DriveStraightOpenLoop(drivetrainSubsystem).withTimeout(3.5)
        ),
        new TurnToAngle(drivetrainSubsystem, ()->0 , ()->0, 180).withTimeout(1),
        new ParallelCommandGroup(
          new Shoot(shooterSubsystem, indexerSubsystem, hoodSubsystem)
        ).withTimeout(6)
    );
  }

  public SequentialCommandGroup getAuto() {
    return auto;
  }
}
