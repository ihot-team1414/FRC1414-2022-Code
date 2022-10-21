package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Drive extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem = DrivetrainSubsystem.getInstance();
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;
  private final BooleanSupplier isFastSupplier;
  private final BooleanSupplier isSlowSupplier;

  public Drive(
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier isFastSupplier,
      BooleanSupplier isSlowSupplier) {
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;
    this.isSlowSupplier = isSlowSupplier;
    this.isFastSupplier = isFastSupplier;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void execute() {
    double translationXPercent = translationXSupplier.getAsDouble();
    double translationYPercent = translationYSupplier.getAsDouble();
    double rotationPercent = rotationSupplier.getAsDouble();
    boolean isSlow = isSlowSupplier.getAsBoolean();
    boolean isFast = isFastSupplier.getAsBoolean();
    double fastSpeed = 1;
    double defaultSpeed = 0.75;
    double slowSpeed = 0.5;
    double speedMultiplier = isFast ? fastSpeed : isSlow ? slowSpeed : defaultSpeed;

    drivetrainSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            translationXPercent * speedMultiplier * Constants.DRIVETRAIN_MAX_VEL,
            translationYPercent * speedMultiplier * Constants.DRIVETRAIN_MAX_VEL,
            rotationPercent * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            drivetrainSubsystem.getGyroscopeRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}
