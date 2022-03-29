package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class Layup extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final TurretSubsystem turretSubsystem;

  public Layup(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem, HoodSubsystem hoodSubsystem, TurretSubsystem turretSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    addRequirements(shooterSubsystem, indexerSubsystem, hoodSubsystem, turretSubsystem);
  }

  @Override
  public void execute() {
    shooterSubsystem.layup();
    hoodSubsystem.set(0.22);
    turretSubsystem.home();
    if (shooterSubsystem.isWithinAllowedError()) {
      indexerSubsystem.load();
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.shooterSubsystem.stop();
    this.indexerSubsystem.stop();
  }
}