package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class Deload extends CommandBase {
  private final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
  private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

  public Deload() {
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
