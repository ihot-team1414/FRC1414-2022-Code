package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.Limelight;
import frc.util.ShooterData;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX shooterMotor1 = new TalonFX(Constants.SHOOTER_ID_1);
  private final TalonFX shooterMotor2 = new TalonFX(Constants.SHOOTER_ID_2);

  public double dashboardTarget = 0;

  private double speed = 0;

  public ShooterSubsystem() {
    shooterMotor1.selectProfileSlot(0, 0);

    shooterMotor1.config_kF(0, Constants.SHOOTER_kF, 0);
    shooterMotor1.config_kP(0, Constants.SHOOTER_kP, 0);
    shooterMotor1.config_kI(0, Constants.SHOOTER_kI, 0);
    shooterMotor1.config_kD(0, Constants.SHOOTER_kD, 0);
    shooterMotor1.enableVoltageCompensation(true);

    shooterMotor1.configPeakOutputForward(Constants.SHOOTER_MAX_OUTPUT);
    shooterMotor2.follow(shooterMotor1);
    shooterMotor1.setInverted(true);
    shooterMotor1.setNeutralMode(NeutralMode.Coast);
    shooterMotor2.setNeutralMode(NeutralMode.Coast);
    shooterMotor2.enableVoltageCompensation(true);
    SmartDashboard.putNumber("Dashboard Shooter Target", 9000);
  }

  public void shoot() {
    double ty = Limelight.getInstance().getDeltaY();

    if (Limelight.getInstance().detectsTarget()) {
      speed = ShooterData.getInstance().getShooterSpeed(ty);
    }
    // 21700 is max theoretical speed for shooter.
    shooterMotor1.set(ControlMode.Velocity, speed);
  }

  public void layup() {
    speed = 7500;
    shooterMotor1.set(ControlMode.Velocity, 7500);
  }

  public boolean isWithinAllowedError() {
    SmartDashboard.putNumber("Shooter Error", shooterMotor1.getClosedLoopError());
    // return Math.abs(shooterMotor1.getClosedLoopError()) < Constants.SHOOTER_ALLOWED_ERROR;
    return Math.abs(shooterMotor1.getSelectedSensorVelocity() - speed) < 0.02 * speed;
  }

  public void eject() {
    shooterMotor1.set(ControlMode.Velocity, 5000);
  }

  public void stop() {
    shooterMotor1.set(ControlMode.PercentOutput, 0.0);
    speed = 0;
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("Shooter Target", shooterMotor1.getClosedLoopTarget());
    SmartDashboard.putNumber("Limelight Distance", Limelight.getInstance().getDeltaY());
    SmartDashboard.putNumber("Shooter Velocity", shooterMotor1.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Shooter Closed Loop Error", shooterMotor1.getClosedLoopError());
    dashboardTarget = SmartDashboard.getNumber("Dashboard Shooter Target", 0.0);
  }
}