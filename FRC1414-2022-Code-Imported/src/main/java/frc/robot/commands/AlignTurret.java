package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class AlignTurret extends CommandBase {
  private final TurretSubsystem turretSubsystem;

  public AlignTurret(TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;

    addRequirements(turretSubsystem);
  }

  @Override
  public void execute() {
    turretSubsystem.visionTargeting();
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.home();
  }
}
