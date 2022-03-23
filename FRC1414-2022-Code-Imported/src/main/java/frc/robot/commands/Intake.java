package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends CommandBase {
  private final IndexerSubsystem indexerSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  public Intake(IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem) {
    this.indexerSubsystem = indexerSubsystem;
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(indexerSubsystem, intakeSubsystem);
  }

  @Override
  public void execute() {
    intakeSubsystem.open();
    intakeSubsystem.intake();
    indexerSubsystem.holdBalls();
  }

  @Override
  public void end(boolean interrupted) {
    this.intakeSubsystem.stop();
    this.intakeSubsystem.close();
    this.indexerSubsystem.stop();
  }
}