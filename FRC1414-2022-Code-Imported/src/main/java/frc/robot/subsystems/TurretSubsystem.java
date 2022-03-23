package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.Limelight;

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
    turretMotor.config_kF(0, Constants.TURRET_MOTOR_kF, 0);
    turretMotor.config_kP(0, Constants.TURRET_MOTOR_kP, 0);
    turretMotor.config_kI(0, Constants.TURRET_MOTOR_kI, 0);
    turretMotor.config_kD(0, Constants.TURRET_MOTOR_kD, 0);

    turretMotor.configMotionCruiseVelocity(Constants.TURRET_MAX_VEL, 30);
    turretMotor.configMotionAcceleration(Constants.TURRET_ACCEL, 30);
  }

  public void moveTurret(double position) {
    if (Constants.TURRET_MAX_POS > position && position > Constants.TURRET_MIN_POS) {
      turretMotor.set(ControlMode.Position, position);
    }
  }

  public void home() {
    moveTurret(0);
  }

  public void eject() {
    moveTurret(4000);
  }

  public void visionTargeting() {
    double deltaX = Limelight.getInstance().getDeltaX();
    double deltaPos = Constants.TURRET_VISION_kP * -deltaX;

    moveTurret(turretMotor.getSelectedSensorPosition() + deltaPos);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Target", turretMotor.getClosedLoopTarget());
    SmartDashboard.putNumber("Turret Position", turretMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Turret Closed Loop Error", turretMotor.getClosedLoopError());
  }
}