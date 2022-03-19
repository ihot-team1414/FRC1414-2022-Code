package frc.robot.subsystems;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class TurretSubsystem extends SubsystemBase {
  private TalonFX turretMotor = new TalonFX(Constants.TURRET_MOTOR_ID);

  public TurretSubsystem() {
    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.configNominalOutputForward(0, 0);
    turretMotor.configNominalOutputReverse(0, 0);
    turretMotor.configPeakOutputForward(Constants.TURRET_MOTOR_MAX_OUTPUT, 0);
    turretMotor.configPeakOutputReverse(-Constants.TURRET_MOTOR_MAX_OUTPUT, 0);
    turretMotor.selectProfileSlot(0, 0);
    turretMotor.config_kF(0, Constants.TURRET_ARM_MOTOR_kF, 0);
    turretMotor.config_kP(0, Constants.TURRET_ARM_MOTOR_kP, 0);
    turretMotor.config_kI(0, Constants.TURRET_ARM_MOTOR_kI, 0);
    turretMotor.config_kD(0, Constants.TURRET_ARM_MOTOR_kD, 0);

    turretMotor.configMotionCruiseVelocity(Constants.TURRET_ARM_MAX_VEL, 30);
    turretMotor.configMotionAcceleration(Constants.TURRET_ARM_ACCEL, 30);
    setVisionMode(true);
  }

  public void moveTurret(double throttle) {
    if (Constants.TURRET_MAX_POS > getEncoder() && getEncoder() > Constants.TURRET_MIN_POS) {
      turretMotor.set(ControlMode.PercentOutput, throttle);
    } else if (Constants.TURRET_MAX_POS <= getEncoder()) {
      if (throttle > 0) {
        turretMotor.set(ControlMode.PercentOutput, 0);
      } else {
        turretMotor.set(ControlMode.PercentOutput, throttle);
      }
    } else if (Constants.TURRET_MIN_POS >= getEncoder()) {
      if (throttle > 0) {
        turretMotor.set(ControlMode.PercentOutput, throttle);
      } else {
        turretMotor.set(ControlMode.PercentOutput, 0);
      }
    }
  }
  
  public double getEncoder() {
    return turretMotor.getSelectedSensorPosition();
  }

  public void resetEncoder() {
    turretMotor.setSelectedSensorPosition(0, 0, 0);
  }

  public void resetPosition() {
    turretMotor.set(ControlMode.Position, 0);
  }

  public void ejectPosition() {
    turretMotor.set(ControlMode.Position, 4000);
  }

  public void visionTargeting() {
    NetworkTableInstance.getDefault().startClientTeam(1414);
    NetworkTableInstance.getDefault().startDSClient();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");

    double error = tx.getDouble(0.0);
    double speed = Constants.TURRET_MOTOR_kP * -error;

    this.moveTurret(speed);
    SmartDashboard.putNumber("Turret Percent Output", speed);
  }

  public void setVisionMode(boolean on) {
    NetworkTableInstance.getDefault().startClientTeam(1414);
    NetworkTableInstance.getDefault().startDSClient();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry camMode = table.getEntry("camMode");
    NetworkTableEntry ledMode = table.getEntry("ledMode");
    
    if (on) {
      camMode.setDouble(0);
      ledMode.setDouble(3);
    } else {
      camMode.setDouble(1);
      ledMode.setDouble(1);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Position", this.getEncoder());
  }
}