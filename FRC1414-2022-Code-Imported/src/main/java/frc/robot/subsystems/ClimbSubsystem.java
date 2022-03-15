package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

  private final TalonFX elevatorMotor = new TalonFX(Constants.CLIMB_TELESCOPING_ARM_2_MOTOR_ID);
  private final TalonFX elevatorMotor2 = new TalonFX(Constants.CLIMB_TELESCOPING_ARM_1_MOTOR_ID);
  private final TalonFX armMotor = new TalonFX(Constants.CLIMB_ARM_1_MOTOR_ID);
  private final TalonFX armMotor2 = new TalonFX(Constants.CLIMB_ARM_2_MOTOR_ID);

  public static enum ArmState {
    Raising, Lowering, Stationary, BottomedOut, ToppedOut,
  }

  public ArmState getArmState() {
      return armState;
  }

  private void setArmState(ArmState newState) {
      this.armState= newState;
  }

  private volatile ArmState armState = ArmState.Stationary;

  public enum ArmPosition {
      Starting(-2000), Vertical(-70000), Grabbing(-60000), Tilting(-100000);
      
      private int position;

      ArmPosition(int encPos) {
          this.position = encPos; // the position is set to the encoder value (which is associated with a position)
      }

      public int getPosition() {
          return this.position; //retrieving the position
      }
  }

  private volatile ArmPosition armPosition = ArmPosition.Starting;

  public ArmPosition getArmPosition() {
      return armPosition;
  }

  public void setArmPosition(ArmPosition newPos) {
      this.armPosition = newPos;
  }

  public enum ElevatorPosition {
      Starting(3000), Intermediate(-97081), Extended(-197081);
      
      private int position;

      ElevatorPosition(int encPos) {
          this.position = encPos; // the position is set to the encoder value (which is associated with a position)
      }

      public int getPosition() {
          return this.position; //retrieving the position
      }
  }

  private volatile ElevatorPosition elevatorPosition = ElevatorPosition.Starting;

  public ElevatorPosition getElevatorPosition() {
      return elevatorPosition;
  }

  public void setElevatorPosition(ElevatorPosition newPos) {
      elevatorPosition = newPos;
  }

  public ClimbSubsystem() {
    this.elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    this.elevatorMotor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    this.elevatorMotor.setNeutralMode(NeutralMode.Brake);
    this.elevatorMotor2.setNeutralMode(NeutralMode.Brake);

    this.elevatorMotor.configNominalOutputForward(0, 0);
    this.elevatorMotor.configNominalOutputReverse(0, 0);
    this.elevatorMotor.configPeakOutputForward(Constants.TELESCOPING_ARM_MOTOR_MAX_OUTPUT, 0);
    this.elevatorMotor.configPeakOutputReverse(-Constants.TELESCOPING_ARM_MOTOR_MAX_OUTPUT, 0);

    this.elevatorMotor.selectProfileSlot(0, 0);
    this.elevatorMotor.config_kF(0, Constants.TELESCOPING_ARM_MOTOR_kF, 0);
	this.elevatorMotor.config_kP(0, Constants.TELESCOPING_ARM_MOTOR_kP, 0);
	this.elevatorMotor.config_kI(0, Constants.TELESCOPING_ARM_MOTOR_kI, 0);
	this.elevatorMotor.config_kD(0, Constants.TELESCOPING_ARM_MOTOR_kD, 0);

    this.elevatorMotor.configMotionCruiseVelocity(10000, 30);
    this.elevatorMotor.configMotionAcceleration(3000, 30);

    this.elevatorMotor2.configNominalOutputForward(0, 0);
    this.elevatorMotor2.configNominalOutputReverse(0, 0);
    this.elevatorMotor2.configPeakOutputForward(Constants.TELESCOPING_ARM_MOTOR_MAX_OUTPUT, 0);
    this.elevatorMotor2.configPeakOutputReverse(-Constants.TELESCOPING_ARM_MOTOR_MAX_OUTPUT, 0);

    this.elevatorMotor2.selectProfileSlot(0, 0);
    this.elevatorMotor2.config_kF(0, Constants.TELESCOPING_ARM_MOTOR_kF, 0);
	this.elevatorMotor2.config_kP(0, Constants.TELESCOPING_ARM_MOTOR_kP, 0);
	this.elevatorMotor2.config_kI(0, Constants.TELESCOPING_ARM_MOTOR_kI, 0);
	this.elevatorMotor2.config_kD(0, Constants.TELESCOPING_ARM_MOTOR_kD, 0);

    this.elevatorMotor2.configMotionCruiseVelocity(15000, 30);
    this.elevatorMotor2.configMotionAcceleration(3000, 30);

    this.armMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    this.armMotor.setNeutralMode(NeutralMode.Brake);

    this.armMotor.configNominalOutputForward(0, 0);
    this.armMotor.configNominalOutputReverse(0, 0);
    this.armMotor.configPeakOutputForward(Constants.PIVOT_ARM_MOTOR_MAX_OUTPUT, 0);
    this.armMotor.configPeakOutputReverse(-Constants.PIVOT_ARM_MOTOR_MAX_OUTPUT, 0);
    
    this.armMotor.configAllowableClosedloopError(0, 0, 0);

    this.armMotor.selectProfileSlot(0, 0);
    this.armMotor.config_kF(0, Constants.PIVOT_ARM_MOTOR_kF, 0);
	this.armMotor.config_kP(0, Constants.PIVOT_ARM_MOTOR_kP, 0);
	this.armMotor.config_kI(0, Constants.PIVOT_ARM_MOTOR_kI, 0);
	this.armMotor.config_kD(0, Constants.PIVOT_ARM_MOTOR_kD, 0);

    this.armMotor.configMotionCruiseVelocity(20000, 30);
    this.armMotor.configMotionAcceleration(5000, 30);
    

    this.armMotor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    this.armMotor2.setNeutralMode(NeutralMode.Brake);

    this.armMotor2.configNominalOutputForward(0, 0);
    this.armMotor2.configNominalOutputReverse(0, 0);
    this.armMotor2.configPeakOutputForward(Constants.PIVOT_ARM_MOTOR_MAX_OUTPUT, 0);
    this.armMotor2.configPeakOutputReverse(-Constants.PIVOT_ARM_MOTOR_MAX_OUTPUT, 0);
    
    this.armMotor2.configAllowableClosedloopError(0, 0, 0);

    this.armMotor2.selectProfileSlot(0, 0);
    this.armMotor2.config_kF(0, Constants.PIVOT_ARM_MOTOR_kF, 0);
	this.armMotor2.config_kP(0, Constants.PIVOT_ARM_MOTOR_kP, 0);
	this.armMotor2.config_kI(0, Constants.PIVOT_ARM_MOTOR_kI, 0);
	this.armMotor2.config_kD(0, Constants.PIVOT_ARM_MOTOR_kD, 0);

    this.armMotor2.configMotionCruiseVelocity(10000, 30);
    this.armMotor2.configMotionAcceleration(3000, 30);

    // startArmMotionMagic(ArmPosition.Vertical);
    // startElevatorMotionMagic(ElevatorPosition.Starting);

  }

  public void startArmMotionMagic(ArmPosition pos) {
    this.armMotor.set(ControlMode.MotionMagic, -pos.getPosition());
    this.armMotor2.set(ControlMode.MotionMagic, pos.getPosition());
  }

  public void stopArm() {
      this.armMotor.set(ControlMode.MotionMagic, 0.0);
  }

  public void directArm(double pow) {
      if (getArmState() == ArmState.BottomedOut && pow < 0.0) {
          return;
      }
      if (getArmState() == ArmState.ToppedOut && pow > 0.0) {
          return;
      }
      if (pow > 0.0) {
          setArmState(ArmState.Raising);
      }
      if (pow < 0.0) {
          setArmState(ArmState.Lowering);
      }
      if (pow == 0.0) {
          setArmState(ArmState.Stationary);
      }
      this.armMotor.set(ControlMode.PercentOutput, pow);
  }

  public double getArmEncoder() {
    return this.armMotor.getSelectedSensorPosition();
  }

  public void startElevatorMotionMagic(ElevatorPosition pos) {
    this.elevatorMotor.set(ControlMode.Position, pos.getPosition());
    this.elevatorMotor2.set(ControlMode.Position, -pos.getPosition());
  }

  public void stopElevator() {
      this.elevatorMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public double getElevatorEncoder() {
    return this.elevatorMotor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Encoder", this.getArmEncoder());
    SmartDashboard.putNumber("Arm Encoder Left", this.armMotor2.getSelectedSensorPosition());

    SmartDashboard.putNumber("Elevator Encoder", this.getElevatorEncoder());
  }
}