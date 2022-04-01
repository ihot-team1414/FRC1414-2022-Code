// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class FollowTrajectory extends CommandBase {

  private Trajectory trajectory;
  private DrivetrainSubsystem drivetrain;

  private boolean reverseRotation = false;
  private static HolonomicDriveController controller;
  private static Timer timer = new Timer();

  PIDController xController = new PIDController(Constants.DRIVETRAIN_PATH_X_kP, 0, 0);
  PIDController yController = new PIDController(Constants.DRIVETRAIN_PATH_Y_kP, 0, 0);

  ProfiledPIDController thetaController = new ProfiledPIDController(
  Constants.DRIVETRAIN_PATH_THETA_kP, Constants.DRIVETRAIN_PATH_THETA_kI, Constants.DRIVETRAIN_PATH_THETA_kD, Constants.THETA_CONTROLLER_CONSTRAINTS);

  public FollowTrajectory(DrivetrainSubsystem drivetrainSubsystem, Trajectory trajectory) {
    this(drivetrainSubsystem, trajectory, false);
  }

  /** Creates a new FollowTrajectory. */
  public FollowTrajectory(DrivetrainSubsystem drivetrain, Trajectory trajectory, boolean reverseRotation) {
    addRequirements(drivetrain);

    this.trajectory = trajectory;
    this.drivetrain = drivetrain;
    this.reverseRotation = reverseRotation;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    controller =
        new HolonomicDriveController(
            xController, yController, thetaController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds speeds =
        controller.calculate(drivetrain.getPose(), trajectory.sample(timer.get()), trajectory.getStates().get(trajectory.getStates().size()-1).poseMeters.getRotation());
    

    ChassisSpeeds invertedSpeeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, drivetrain.getRequiredTurningSpeedForAngle(trajectory.getStates().get(trajectory.getStates().size()-1).poseMeters.getRotation().getDegrees(), reverseRotation));
    drivetrain.drive(invertedSpeeds);
    SmartDashboard.putBoolean("Auto?", true);
    SmartDashboard.putNumber("timer", timer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0,0,0));
    SmartDashboard.putBoolean("AUto?", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > trajectory.getTotalTimeSeconds();
  }
}