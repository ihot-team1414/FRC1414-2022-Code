package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.util.Limelight;
import frc.util.ShooterData;

public class Shoot extends CommandBase {
  private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
  private final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
  private final HoodSubsystem hoodSubsystem = HoodSubsystem.getInstance();
  private final XboxController operatorController;

  private double startTime;
  private double speed;
  private boolean withinError = false;
  private boolean linedUp = false;

  public Shoot(XboxController operatorController) {
    startTime = Timer.getFPGATimestamp();
    withinError = false;
    addRequirements(shooterSubsystem, indexerSubsystem, hoodSubsystem);
    this.operatorController = operatorController;
  }

  @Override
  public void initialize() {
    super.initialize();
    startTime = Timer.getFPGATimestamp();
    withinError = false;
    linedUp = false;

    if (Constants.MANUAL_SPEED_AND_ANGLE) {
      speed = SmartDashboard.getNumber("Dashboard Shooter Target", 0.0);
      hoodSubsystem.set(SmartDashboard.getNumber("Dashboard Hood Target", 0.0));
    } else if (Limelight.getInstance().detectsTarget()) {
      double ty = Limelight.getInstance().getDeltaY();
      speed = ShooterData.getInstance().getShooterSpeed(ty);
      hoodSubsystem.set(ShooterData.getInstance().getHoodAngle(ty));
    } else {
      speed = Constants.SHOOTER_DEFAULT_SPEED;
      hoodSubsystem.set(Constants.HOOD_DEFAULT_ANGLE);
    }
  }

  @Override
  public void execute() {
    shooterSubsystem.shoot(speed);

    if (!Constants.MANUAL_SPEED_AND_ANGLE && !linedUp && TurretSubsystem.getInstance().isWithinAllowedVisionError()) {
      double ty = Limelight.getInstance().getDeltaY();
      speed = ShooterData.getInstance().getShooterSpeed(ty);
      hoodSubsystem.set(ShooterData.getInstance().getHoodAngle(ty));
      linedUp = true;
    }

    // RPM + Time Based Indexing
    if (linedUp && operatorController.getRightTriggerAxis() > 0.5) {
      if (!Constants.MANUAL_SPEED_AND_ANGLE && !TurretSubsystem.getInstance().isWithinAllowedVisionError()) {
        linedUp = false;
      } else if (!withinError && shooterSubsystem.isWithinAllowedError()) {
        startTime = Timer.getFPGATimestamp();
        withinError = true;
      } else if (withinError && !shooterSubsystem.isWithinAllowedError()) {
        withinError = false;
      } else if (withinError && Timer.getFPGATimestamp() - startTime > 0.05) {
        indexerSubsystem.load();
      } else {
        indexerSubsystem.stop();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    speed = 0;
    withinError = false;
    shooterSubsystem.stop();
    indexerSubsystem.stop();
  }
}