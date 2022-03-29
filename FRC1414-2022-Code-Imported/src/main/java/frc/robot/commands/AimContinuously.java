package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ClimbSubsystem.PivotPosition;
import frc.util.Limelight;

public class AimContinuously extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final ClimbSubsystem climbSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final PIDController rotationController;

  public AimContinuously(DrivetrainSubsystem drivetrainSubsystem, ClimbSubsystem climbSubsystem, TurretSubsystem turretSubsystem, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.climbSubsystem = climbSubsystem;
    this.turretSubsystem = turretSubsystem;

    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;

    rotationController = new PIDController(Constants.DRIVETRAIN_VISION_ROTATION_kP, 0, 0);

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void execute() {
    double translationXPercent = translationXSupplier.getAsDouble() * 0.75 * Constants.DRIVETRAIN_MAX_VEL;
    double translationYPercent = translationYSupplier.getAsDouble() * 0.75 * Constants.DRIVETRAIN_MAX_VEL;

    if (!climbSubsystem.isPivotAtTarget(PivotPosition.Vertical)) {
      climbSubsystem.setPivot(PivotPosition.Vertical);
      turretSubsystem.home();
    } else {
      turretSubsystem.visionTargeting();
    }

    double rotation = rotationController.calculate(turretSubsystem.getPosition(), 0);

    if (!Limelight.getInstance().detectsTarget()) {
      rotation = 6;
    }

    SmartDashboard.putNumber("Auto Drive Rotation Percent", rotation);

    drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(translationYPercent, translationXPercent, rotation, drivetrainSubsystem.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }
}
