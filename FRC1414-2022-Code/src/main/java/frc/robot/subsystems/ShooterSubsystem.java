package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase{
  public final TalonFX shooterMotor1 = new TalonFX(Constants.SHOOTER_ID_1);
  public final TalonFX shooterMotor2 = new TalonFX(Constants.SHOOTER_ID_2);


  public Shooter() {
    this.shooterMotor1.setInverted(true);
    this.shooterMotor2.setInverted(false);
    this.shooterMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0); // encoder
    this.shooterMotor2.follow(shooterMotor1);

    this.shooterMotor1.setNeutralMode(NeutralMode.Coast);
    this.shooterMotor2.setNeutralMode(NeutralMode.Coast);

    this.shooterMotor1.config_kF(Constants.SHOOTER_PID_LOOP_ID, Constants.SHOOTER_kF, Constants.SHOOTER_TIMEOUT_MS);
    this.shooterMotor1.config_kP(Constants.SHOOTER_PID_LOOP_ID, Constants.SHOOTER_kP, Constants.SHOOTER_TIMEOUT_MS);
    this.shooterMotor1.config_kI(Constants.SHOOTER_PID_LOOP_ID, Constants.SHOOTER_kI, Constants.SHOOTER_TIMEOUT_MS);
    this.shooterMotor1.config_kD(Constants.SHOOTER_PID_LOOP_ID, Constants.SHOOTER_kD, Constants.SHOOTER_TIMEOUT_MS);
  }

  public void shoot() {
    double outputVelocity = (Constants.SHOOTER_RPM * 2048 / 600); // todo
    this.shooterMotor1.set(ControlMode.Velocity, outputVelocity);
  }

  public void lowGoal() {
    double outputVelocity = Constants.LOW_GOAL_RPM * 2048 / 600;
    this.shooterMotor1.set(ControlMode.Velocity, outputVelocity);
  }

  public double getVelocity() {
    return this.shooterMotor1.getSelectedSensorVelocity();
  }

  public void stop() {
    this.shooterMotor1.set(ControlMode.PercentOutput, 0.0);
  }

  public double getActualRPM() {
    return this.getVelocity() * 600 / 2048; // TODO
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed", this.getVelocity());
    SmartDashboard.putNumber("Shooter Actual RPM", this.getActualRPM());
  }

}// end of class