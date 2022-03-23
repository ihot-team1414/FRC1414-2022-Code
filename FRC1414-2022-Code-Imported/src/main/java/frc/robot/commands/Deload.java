package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;


public class Deload extends CommandBase {
  private final IndexerSubsystem indexerSubsystem;

  public Deload(IndexerSubsystem indexerSubsystem) {
    this.indexerSubsystem = indexerSubsystem;

    addRequirements(indexerSubsystem);
  }

  @Override
  public void execute() {
    indexerSubsystem.reverse();
  }

  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.stop();
  }
}
