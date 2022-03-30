package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
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

  private double startTime = Timer.getFPGATimestamp();

  public Layup(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem, HoodSubsystem hoodSubsystem, TurretSubsystem turretSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    addRequirements(shooterSubsystem, indexerSubsystem, hoodSubsystem, turretSubsystem);
  }

  @Override
  public void initialize() {
    // TODO Auto-generated method stub
    super.initialize();
    this.startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    hoodSubsystem.set(0.28);
    turretSubsystem.home();

    shooterSubsystem.layup();

    if (Timer.getFPGATimestamp() - this.startTime > 0.5) {
      if (shooterSubsystem.isWithinAllowedError()) {
        indexerSubsystem.load();
      }
    } 
  }

  @Override
  public void end(boolean interrupted) {
    this.shooterSubsystem.stop();
    this.indexerSubsystem.stop();
  }
}