package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LoadBalls extends CommandBase {
  private final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
  private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

  public LoadBalls() {
    // We exclude the shooter subsystem from requirements because we only access sensor data.
    addRequirements(indexerSubsystem);
  }

  @Override
  public void execute() {
    if (shooterSubsystem.getShooterSpeed() > Constants.SHOOTER_MIN_LOAD_SPEED) {
      indexerSubsystem.load();
    }
  }

  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.stop();
  }
}