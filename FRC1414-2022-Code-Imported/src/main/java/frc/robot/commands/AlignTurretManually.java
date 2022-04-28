package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.util.Utils;

public class AlignTurretManually extends CommandBase {
  private final TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();
  private final DoubleSupplier translationXSupplier;

  public AlignTurretManually(DoubleSupplier translationXSupplier) {
    this.translationXSupplier = translationXSupplier;
    addRequirements(turretSubsystem);
  }

  @Override
  public void execute() {
    turretSubsystem.setTurret(Utils.deadband(0.4 * translationXSupplier.getAsDouble(), 0.1));
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.zero();
    turretSubsystem.home();
  }
}
