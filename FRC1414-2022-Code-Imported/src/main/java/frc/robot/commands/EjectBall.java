package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class EjectBall extends CommandBase {
  private final TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();
  private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
  private final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();

  public EjectBall() {
    addRequirements(turretSubsystem, shooterSubsystem, indexerSubsystem);
  }

  @Override
  public void execute() {
    turretSubsystem.eject();

    if (turretSubsystem.isWithinAllowedError()) {
      shooterSubsystem.eject();
      indexerSubsystem.load();
    }
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.home();
    shooterSubsystem.stop();
    indexerSubsystem.stop();
  }
}
