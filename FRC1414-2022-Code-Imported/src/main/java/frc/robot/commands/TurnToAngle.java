package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnToAngle extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final double angle;

  public TurnToAngle(DrivetrainSubsystem drivetrainSubsystem, double angle) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.angle = angle;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void execute() {
    drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, drivetrainSubsystem.getRequiredTurningSpeedForAngle(angle)));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }
}
