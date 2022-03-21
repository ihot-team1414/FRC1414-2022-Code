package frc.robot.subsystems;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

  private final TalonFX telescopingMotor = new TalonFX(Constants.TELESCOPING_ARM_1_MOTOR_ID);
  private final TalonFX telescopingMotor2 = new TalonFX(Constants.TELESCOPING_ARM_2_MOTOR_ID);
  private final TalonFX pivotMotor = new TalonFX(Constants.PIVOT_ARM_1_MOTOR_ID);
  private final TalonFX pivotMotor2 = new TalonFX(Constants.PIVOT_ARM_2_MOTOR_ID);

  public enum ArmPosition {
      Starting(-2000), Vertical(-70000), Grabbing(-60000), Tilting(-100000), Lifting(-40000);
      
      private int position;

      ArmPosition(int encPos) {
          this.position = encPos; // the position is set to the encoder value (which is associated with a position)
      }

      public int getPosition() {
          return this.position; //retrieving the position
      }
  }

  public enum ElevatorPosition {
      Starting(3000), Neutral(0), Intermediate(-97081), FirstRung(-150000), Extended(-190000);
      
      private int position;

      ElevatorPosition(int encPos) {
          this.position = encPos; // the position is set to the encoder value (which is associated with a position)
      }

      public int getPosition() {
          return this.position; //retrieving the position
      }
  }

  public ClimbSubsystem() {
    
    // TELESCOPING ARM 1
    telescopingMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    telescopingMotor.setNeutralMode(NeutralMode.Brake);

    telescopingMotor.configNominalOutputForward(0, 0);
    telescopingMotor.configNominalOutputReverse(0, 0);
    telescopingMotor.configPeakOutputForward(Constants.TELESCOPING_ARM_MOTOR_MAX_OUTPUT, 0);
    telescopingMotor.configPeakOutputReverse(-Constants.TELESCOPING_ARM_MOTOR_MAX_OUTPUT, 0);

    telescopingMotor.configAllowableClosedloopError(0, Constants.TELESCOPING_ARM_ALLOWED_ERROR, 30);

    telescopingMotor.selectProfileSlot(0, 0);
    telescopingMotor.config_kF(0, Constants.TELESCOPING_ARM_MOTOR_kF, 0);
	telescopingMotor.config_kP(0, Constants.TELESCOPING_ARM_MOTOR_kP, 0);
	telescopingMotor.config_kI(0, Constants.TELESCOPING_ARM_MOTOR_kI, 0);
	telescopingMotor.config_kD(0, Constants.TELESCOPING_ARM_MOTOR_kD, 0);

    telescopingMotor.configMotionCruiseVelocity(Constants.TELESCOPING_ARM_MAX_VEL, 30);
    telescopingMotor.configMotionAcceleration(Constants.TELESCOPING_ARM_ACCEL, 30);


    // TELESCOPING ARM 2
    telescopingMotor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    telescopingMotor2.setNeutralMode(NeutralMode.Brake);

    telescopingMotor2.configNominalOutputForward(0, 0);
    telescopingMotor2.configNominalOutputReverse(0, 0);
    telescopingMotor2.configPeakOutputForward(Constants.TELESCOPING_ARM_MOTOR_MAX_OUTPUT, 0);
    telescopingMotor2.configPeakOutputReverse(-Constants.TELESCOPING_ARM_MOTOR_MAX_OUTPUT, 0);

    telescopingMotor2.configAllowableClosedloopError(0, Constants.TELESCOPING_ARM_ALLOWED_ERROR, 0);

    telescopingMotor2.selectProfileSlot(0, 0);
    telescopingMotor2.config_kF(0, Constants.TELESCOPING_ARM_MOTOR_kF, 0);
	telescopingMotor2.config_kP(0, Constants.TELESCOPING_ARM_MOTOR_kP, 0);
	telescopingMotor2.config_kI(0, Constants.TELESCOPING_ARM_MOTOR_kI, 0);
	telescopingMotor2.config_kD(0, Constants.TELESCOPING_ARM_MOTOR_kD, 0);

    telescopingMotor2.configMotionCruiseVelocity(Constants.TELESCOPING_ARM_MAX_VEL, 30);
    telescopingMotor2.configMotionAcceleration(Constants.TELESCOPING_ARM_ACCEL, 30);


    // PIVOT ARM 1
    pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    pivotMotor.setNeutralMode(NeutralMode.Brake);

    pivotMotor.configNominalOutputForward(0, 0);
    pivotMotor.configNominalOutputReverse(0, 0);
    pivotMotor.configPeakOutputForward(Constants.PIVOT_ARM_MOTOR_MAX_OUTPUT, 0);
    pivotMotor.configPeakOutputReverse(-Constants.PIVOT_ARM_MOTOR_MAX_OUTPUT, 0);
    
    pivotMotor.configAllowableClosedloopError(0, 0, 0);

    pivotMotor.selectProfileSlot(0, 0);
    pivotMotor.config_kF(0, Constants.PIVOT_ARM_MOTOR_kF, 0);
	pivotMotor.config_kP(0, Constants.PIVOT_ARM_MOTOR_kP, 0);
	pivotMotor.config_kI(0, Constants.PIVOT_ARM_MOTOR_kI, 0);
	pivotMotor.config_kD(0, Constants.PIVOT_ARM_MOTOR_kD, 0);

    pivotMotor.configMotionCruiseVelocity(Constants.PIVOT_ARM_MAX_VEL, 30);
    pivotMotor.configMotionAcceleration(Constants.PIVOT_ARM_ACCEL, 30);
    

    // PIVOT ARM 2
    pivotMotor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    pivotMotor2.setNeutralMode(NeutralMode.Brake);

    pivotMotor2.configNominalOutputForward(0, 0);
    pivotMotor2.configNominalOutputReverse(0, 0);
    pivotMotor2.configPeakOutputForward(Constants.PIVOT_ARM_MOTOR_MAX_OUTPUT, 0);
    pivotMotor2.configPeakOutputReverse(-Constants.PIVOT_ARM_MOTOR_MAX_OUTPUT, 0);
    
    pivotMotor2.configAllowableClosedloopError(0, 0, 0);

    pivotMotor2.selectProfileSlot(0, 0);
    pivotMotor2.config_kF(0, Constants.PIVOT_ARM_MOTOR_kF, 0);
	pivotMotor2.config_kP(0, Constants.PIVOT_ARM_MOTOR_kP, 0);
	pivotMotor2.config_kI(0, Constants.PIVOT_ARM_MOTOR_kI, 0);
	pivotMotor2.config_kD(0, Constants.PIVOT_ARM_MOTOR_kD, 0);

    pivotMotor2.configMotionCruiseVelocity(Constants.PIVOT_ARM_MAX_VEL, 30);
    pivotMotor2.configMotionAcceleration(Constants.PIVOT_ARM_ACCEL, 30);
  }

  public boolean isElevatorAtTarget(ElevatorPosition position) {
    boolean arm1Aligned = Math.abs(position.getPosition() - telescopingMotor.getSelectedSensorPosition()) <= Constants.TELESCOPING_ARM_ALLOWED_ERROR;
    boolean arm2Aligned = Math.abs(position.getPosition() - telescopingMotor2.getSelectedSensorPosition()) <= Constants.TELESCOPING_ARM_ALLOWED_ERROR;

    return arm1Aligned && arm2Aligned;
}

  public boolean isArmAtTarget(ArmPosition position) {
      boolean arm1Aligned = Math.abs(position.getPosition() - pivotMotor.getSelectedSensorPosition()) <= Constants.PIVOT_ARM_ALLOWED_ERROR;
      boolean arm2Aligned = Math.abs(position.getPosition() - pivotMotor2.getSelectedSensorPosition()) <= Constants.PIVOT_ARM_ALLOWED_ERROR;

      return arm1Aligned && arm2Aligned;
  }

  public void setArmPosition(ArmPosition pos) {
    pivotMotor.set(ControlMode.MotionMagic, -pos.getPosition());
    pivotMotor2.set(ControlMode.MotionMagic, pos.getPosition());
  }

  public void stopArm() {
      pivotMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void setElevatorPosition(ElevatorPosition pos) {
    telescopingMotor.set(ControlMode.Position, pos.getPosition());
    telescopingMotor2.set(ControlMode.Position, -pos.getPosition());
  }

  public void stopElevator() {
      telescopingMotor.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Encoder 1", pivotMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Arm Encoder 2", pivotMotor2.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elevator Encoder 1", telescopingMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elevator Encoder 2", telescopingMotor2.getSelectedSensorPosition());
  }
}