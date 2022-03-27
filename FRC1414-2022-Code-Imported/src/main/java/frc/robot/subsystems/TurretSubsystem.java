package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.Limelight;

public class TurretSubsystem extends SubsystemBase {
  private TalonFX turretMotor = new TalonFX(Constants.TURRET_MOTOR_ID);
  private PIDController visionController = new PIDController(Constants.TURRET_MOTOR_VISTION_kP,
    Constants.TURRET_MOTOR_VISTION_kI, Constants.TURRET_MOTOR_VISTION_kD);

  public TurretSubsystem() {
    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.configNominalOutputForward(0.0, 30);
    turretMotor.configNominalOutputReverse(0.0, 30);
    turretMotor.configPeakOutputForward(Constants.TURRET_MOTOR_MAX_OUTPUT, 30);
    turretMotor.configPeakOutputReverse(-Constants.TURRET_MOTOR_MAX_OUTPUT, 30);
    turretMotor.selectProfileSlot(0, 0);
    turretMotor.config_kF(0, Constants.TURRET_MOTOR_POSITION_kF, 0);
    turretMotor.config_kP(0, Constants.TURRET_MOTOR_POSITION_kP, 0);
    turretMotor.config_kI(0, Constants.TURRET_MOTOR_POSITION_kI, 0);
    turretMotor.config_kD(0, Constants.TURRET_MOTOR_POSITION_kD, 0);
    turretMotor.configAllowableClosedloopError(0, Constants.TURRET_POSITION_ALLOWED_ERROR, 30);
    turretMotor.configForwardSoftLimitThreshold(Constants.TURRET_MAX_POS);
    turretMotor.configReverseSoftLimitThreshold(Constants.TURRET_MIN_POS);
    turretMotor.configReverseSoftLimitEnable(true);
    turretMotor.configForwardSoftLimitEnable(true);
  }

  public void setTurret(double speed) {
    turretMotor.set(ControlMode.PercentOutput, speed);
  }

  public void home() {
    turretMotor.set(ControlMode.Position, 0);
  }

  public void eject() {
    setTurret(4000);
  }

  public boolean isWithinLimits() {
    return turretMotor.getSelectedSensorPosition() > Constants.TURRET_MIN_POS && turretMotor.getSelectedSensorPosition() < Constants.TURRET_MAX_POS;
  }

  public boolean isHome() {
    return turretMotor.getSelectedSensorPosition() < Constants.TURRET_POSITION_ALLOWED_ERROR
      && turretMotor.getSelectedSensorPosition() > -Constants.TURRET_POSITION_ALLOWED_ERROR;
  }

  public boolean isWithinAllowedError() {
    return Math.abs(turretMotor.getClosedLoopError()) < Constants.TURRET_POSITION_ALLOWED_ERROR;
  }

  public void visionTargeting() {
    double deltaX = Limelight.getInstance().getDeltaX();

    if (Math.abs(deltaX) > Constants.TURRET_VISION_ALLOWED_ERROR) {
      setTurret(visionController.calculate(deltaX, 0));
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Target", turretMotor.getClosedLoopTarget());
    SmartDashboard.putNumber("Turret Position", turretMotor.getSelectedSensorPosition());

    SmartDashboard.putNumber("Turret Closed Loop Error", turretMotor.getClosedLoopError());
    SmartDashboard.putNumber("Turret Output Percent", turretMotor.getMotorOutputPercent());
  }
}