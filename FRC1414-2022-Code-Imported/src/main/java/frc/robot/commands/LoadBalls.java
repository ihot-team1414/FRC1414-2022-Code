package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class LoadBalls extends CommandBase {
  private final IndexerSubsystem indexerSubsystem;

  public LoadBalls(IndexerSubsystem indexerSubsystem) {
    this.indexerSubsystem = indexerSubsystem;

    addRequirements(indexerSubsystem);
  }

  @Override
  public void execute() {
    indexerSubsystem.load();
  }

  @Override
  public void end(boolean interrupted) {
    this.indexerSubsystem.stop();
  }
}