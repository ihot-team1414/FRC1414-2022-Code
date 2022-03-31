package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ClimbSubsystem.PivotPosition;
import frc.util.Utils;
import frc.robot.subsystems.ClimbSubsystem;

public class AlignTurretManually extends CommandBase {
  private final TurretSubsystem turretSubsystem;
  private final double translationXPercent;

  public AlignTurretManually(TurretSubsystem turretSubsystem, DoubleSupplier translationXSupplier) {
    this.turretSubsystem = turretSubsystem;
    translationXPercent = translationXSupplier.getAsDouble();

    addRequirements(turretSubsystem);
  }

  @Override
  public void execute() {
    turretSubsystem.setTurret(Utils.deadband(0.2 * translationXPercent, 0.1));
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.zero();
    turretSubsystem.home();
  }
}
