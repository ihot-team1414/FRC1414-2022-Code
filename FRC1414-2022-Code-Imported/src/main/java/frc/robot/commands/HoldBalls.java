package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class HoldBalls extends CommandBase {
  private final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();

  public HoldBalls() {
    addRequirements(indexerSubsystem);
  }

  @Override
  public void execute() {
    indexerSubsystem.holdBalls();
  }

  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.stop();
  }
}