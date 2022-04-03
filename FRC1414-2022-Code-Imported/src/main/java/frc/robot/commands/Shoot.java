package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.util.Limelight;
import frc.util.ShooterData;

public class Shoot extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final HoodSubsystem hoodSubsystem;

  private double startTime;
  private double speed;
  private boolean withinError = false;

  public Shoot(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem, HoodSubsystem hoodSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.startTime = Timer.getFPGATimestamp();
    this.withinError = false;
    addRequirements(shooterSubsystem, indexerSubsystem, hoodSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    startTime = Timer.getFPGATimestamp();
    withinError = false;

    if (Constants.MANUAL_SPEED_AND_ANGLE) {
      speed = SmartDashboard.getNumber("Dashboard Shooter Target", 0.0);
      hoodSubsystem.set(SmartDashboard.getNumber("Dashboard Hood Target", 0.0));
    } else if (Limelight.getInstance().detectsTarget()) {
      double ty = Limelight.getInstance().getDeltaY();
      speed = ShooterData.getInstance().getShooterSpeed(ty);
      hoodSubsystem.set(ShooterData.getInstance().getHoodAngle(ty));
    }
  }

  @Override
  public void execute() {
    shooterSubsystem.shoot(speed);
    SmartDashboard.putNumber("Start Time", startTime);
    SmartDashboard.putNumber("Curr Time", Timer.getFPGATimestamp());
    
    // Time Based Indexing
    // if (Timer.getFPGATimestamp() - startTime > 1.5) {
    //   this.indexerSubsystem.load();
    //   SmartDashboard.putBoolean("hOLDING", false);
    // }
    // // } else if (Timer.getFPGATimestamp() - startTime > 1) {
    // //   this.indexerSubsystem.stop();
    // // } 
    // else {
    //   this.indexerSubsystem.holdBalls();
    //   SmartDashboard.putBoolean("hOLDING", true);
    // }

    // RPM + Time Based Indexing
    if (!withinError && shooterSubsystem.isWithinAllowedError()) {
      startTime = Timer.getFPGATimestamp();
      withinError = true;
    } else if (withinError && !shooterSubsystem.isWithinAllowedError()) {
      withinError = false;
    } else if (withinError && Timer.getFPGATimestamp() - startTime > 0.6) {
      this.indexerSubsystem.load();
      SmartDashboard.putBoolean("hOLDING", false);
    } else if (withinError && Timer.getFPGATimestamp() - startTime > 0.4) {
      this.indexerSubsystem.stop();
      SmartDashboard.putBoolean("hOLDING", false);
    } else {
      this.indexerSubsystem.stop();
      SmartDashboard.putBoolean("hOLDING", true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    speed = 0;
    withinError = false;
    this.shooterSubsystem.stop();
    this.indexerSubsystem.stop();
  }
}