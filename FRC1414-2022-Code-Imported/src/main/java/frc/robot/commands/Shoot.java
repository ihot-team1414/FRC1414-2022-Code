package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  public Shoot(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    addRequirements(shooterSubsystem, indexerSubsystem);
  }

  @Override
  public void execute() {
    shooterSubsystem.shoot();
    
    if (shooterSubsystem.isWithinAllowedError()) {
      indexerSubsystem.load();
    } else {
      // indexerSubsystem.holdBalls();
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.shooterSubsystem.stop();
    this.indexerSubsystem.stop();
  }
}