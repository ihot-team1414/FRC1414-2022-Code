package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class Deload extends CommandBase {
  private final IndexerSubsystem indexerSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  public Deload(IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem) {
    this.indexerSubsystem = indexerSubsystem;
    this.intakeSubsystem = intakeSubsystem; 

    addRequirements(indexerSubsystem, intakeSubsystem);
  }

  @Override
  public void execute() {
    indexerSubsystem.reverse();
    intakeSubsystem.open();
    intakeSubsystem.outtake();
  }

  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.stop();
    intakeSubsystem.close();
    intakeSubsystem.stop();
  }
}
