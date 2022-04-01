package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.util.Utils;

public class AlignTurretManually extends CommandBase {
  private final TurretSubsystem turretSubsystem;
  private final DoubleSupplier translationXSupplier;

  public AlignTurretManually(TurretSubsystem turretSubsystem, DoubleSupplier translationXSupplier) {
    this.turretSubsystem = turretSubsystem;
    this.translationXSupplier = translationXSupplier;
    addRequirements(turretSubsystem);
  }

  @Override
  public void execute() {
    turretSubsystem.setTurret(Utils.deadband(0.4 * translationXSupplier.getAsDouble(), 0.1));
    System.out.println("G3 GIRL!G3 GIRL!G3 GIRL!G3 GIRL!G3 GIRL!G3 GIRL!G3 GIRL!G3 GIRL!");
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.zero();
    turretSubsystem.home();
  }
}
