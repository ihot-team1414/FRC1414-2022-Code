package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveStraightOpenLoop extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;

  public DriveStraightOpenLoop(DrivetrainSubsystem drivetrainSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void execute() {
    drivetrainSubsystem.drive(new ChassisSpeeds(-1.25, 0, 0));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }
}
