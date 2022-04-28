package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAndHold extends CommandBase {
  private final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
  private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

  public IntakeAndHold() {
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
    intakeSubsystem.stop();
    intakeSubsystem.close();
    indexerSubsystem.stop();
  }
}