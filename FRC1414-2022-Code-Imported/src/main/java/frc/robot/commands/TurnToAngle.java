package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnToAngle extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem = DrivetrainSubsystem.getInstance();
  private final double angle;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;

  public TurnToAngle(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, double angle) {
    this.angle = angle;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void execute() {
    double translationXPercent = translationXSupplier.getAsDouble() * 0.75 * Constants.DRIVETRAIN_MAX_VEL;
    double translationYPercent = translationYSupplier.getAsDouble() * 0.75 * Constants.DRIVETRAIN_MAX_VEL;

    drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(translationYPercent, translationXPercent, drivetrainSubsystem.getRequiredTurningSpeedForAngle(angle), drivetrainSubsystem.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }
}
