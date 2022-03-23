package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  public ShooterCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    addRequirements(this.shooterSubsystem, this.indexerSubsystem);
  }

  public void initialize() {
  }

  public void execute() {
    this.shooterSubsystem.shoot();
    
    if (this.shooterSubsystem.getShooterVelocity() > this.shooterSubsystem.getShooterTarget()) {
      this.indexerSubsystem.load();
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.shooterSubsystem.stop();
    this.indexerSubsystem.stop();
}
}