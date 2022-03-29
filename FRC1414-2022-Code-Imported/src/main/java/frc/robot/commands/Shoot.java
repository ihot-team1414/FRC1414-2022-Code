package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  private double startTime;

  public Shoot(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.startTime = Timer.getFPGATimestamp();
    addRequirements(shooterSubsystem, indexerSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    shooterSubsystem.shoot();
    SmartDashboard.putNumber("Start Time", startTime);
    SmartDashboard.putNumber("Curr Time", Timer.getFPGATimestamp());
    
    if (Timer.getFPGATimestamp() - startTime > 1.5) {
      this.indexerSubsystem.load();
      SmartDashboard.putBoolean("hOLDING", false);
    } else if (Timer.getFPGATimestamp() - startTime > 1) {
      this.indexerSubsystem.stop();
    } else {
      this.indexerSubsystem.holdBalls();
      SmartDashboard.putBoolean("hOLDING", true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.shooterSubsystem.stop();
    this.indexerSubsystem.stop();
  }
}