package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private static ShooterSubsystem instance;

  private final TalonFX shooterMotor1 = new TalonFX(Constants.SHOOTER_ID_1);
  private final TalonFX shooterMotor2 = new TalonFX(Constants.SHOOTER_ID_2);

  private double speed = 0;

  public static synchronized ShooterSubsystem getInstance() {
    if (instance == null) {
      instance = new ShooterSubsystem();
    }

    return instance;
  }

  public ShooterSubsystem() {
    shooterMotor1.selectProfileSlot(0, 0);

    shooterMotor1.config_kF(0, Constants.SHOOTER_kF, 0);
    shooterMotor1.config_kP(0, Constants.SHOOTER_kP, 0);
    shooterMotor1.config_kI(0, Constants.SHOOTER_kI, 0);
    shooterMotor1.config_kD(0, Constants.SHOOTER_kD, 0);
    shooterMotor1.enableVoltageCompensation(true);
    shooterMotor1.configVoltageCompSaturation(12.0, 30);
    shooterMotor1.configPeakOutputForward(Constants.SHOOTER_MAX_OUTPUT);
    shooterMotor1.setInverted(true);
    shooterMotor1.setNeutralMode(NeutralMode.Coast);

    shooterMotor2.follow(shooterMotor1);
    shooterMotor2.setNeutralMode(NeutralMode.Coast);
    shooterMotor2.enableVoltageCompensation(true);
    shooterMotor2.configVoltageCompSaturation(12.0, 30);

    if (Constants.MANUAL_SPEED_AND_ANGLE) {
      SmartDashboard.putNumber("Dashboard Shooter Target", 9000);
    }
  }

  public void shoot(double speed) {
    // 21700 is max theoretical speed for shooter.
    this.speed = speed;

    //THERMOSTAT
    // if (shooterMotor1.getSelectedSensorVelocity() >= speed) {
    //   shooterMotor1.set(ControlMode.PercentOutput, 0);
    // } else {
    //   shooterMotor1.set(ControlMode.PercentOutput, 0.25);
    // }

    shooterMotor1.set(ControlMode.Velocity, speed);
    // shooterMotor1.set(ControlMode.PercentOutput, 0.5);
  }

  public void layup() {
    speed = Constants.SHOOTER_LAYUP_SPEED;
    shooterMotor1.set(ControlMode.Velocity, speed);
  }

  public boolean isWithinAllowedError() {
    double allowedError = Constants.SHOOTER_ALLOWED_ERROR;
    if (speed >= 7000) {
      allowedError = Constants.SHOOTER_ALLOWED_ERROR * (1 + ((speed - 7000) / 4000));
    }
    return Math.abs(shooterMotor1.getSelectedSensorVelocity() - speed) < allowedError;
  }

  public double getShooterSpeed() {
    return shooterMotor1.getSelectedSensorVelocity();
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
    SmartDashboard.putNumber("Shooter Velocity", shooterMotor1.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("Shooter Closed Loop Error", shooterMotor1.getClosedLoopError());
  }
}