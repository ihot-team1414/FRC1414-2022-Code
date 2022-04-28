package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShootLowGoal extends CommandBase {

  private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
  private final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
  private final HoodSubsystem hoodSubsystem = HoodSubsystem.getInstance();
  private final TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();

  public ShootLowGoal() {
    addRequirements(shooterSubsystem, indexerSubsystem, hoodSubsystem, turretSubsystem);
  }

  @Override
  public void execute() {
    hoodSubsystem.set(0.28);
    turretSubsystem.home();

    shooterSubsystem.layup();

    if (shooterSubsystem.isWithinAllowedError()) {
      indexerSubsystem.load();
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    indexerSubsystem.stop();
  }
}