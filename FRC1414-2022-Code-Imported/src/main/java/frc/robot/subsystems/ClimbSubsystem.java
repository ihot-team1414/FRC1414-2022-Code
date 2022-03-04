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

  private final TalonFX elevatorMotor = new TalonFX(Constants.CLIMB_TELESCOPING_ARM_1_MOTOR_ID);
  private final TalonFX elevatorMotor2 = new TalonFX(Constants.CLIMB_TELESCOPING_ARM_2_MOTOR_ID);
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
      Starting(0), Vertical(0), Grabbing(0), Tilting(0); // TODO
      
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

  public static enum ElevatorState {
    Raising, Lowering, Stationary, BottomedOut, ToppedOut,
  }

  public ElevatorState getElevatorState() {
      return elevatorState;
  }

  private void setElevatorState(ElevatorState newState) {
      this.elevatorState = newState;
  }

  private volatile ElevatorState elevatorState = ElevatorState.Stationary;

  public enum ElevatorPosition {
      Starting(0), Extended(0); // TODO
      
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
      this.elevatorPosition = newPos;
  }

  public ClimbSubsystem() {

    this.armMotor2.follow(armMotor);
    this.elevatorMotor2.follow(elevatorMotor);


    this.elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

    this.armMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);


  
    this.elevatorMotor.setNeutralMode(NeutralMode.Brake);
    this.armMotor.setNeutralMode(NeutralMode.Brake);

    this.armMotor.configNominalOutputForward(0, 0);
    this.armMotor.configNominalOutputReverse(0, 0);
    this.armMotor.configPeakOutputForward(Constants.PIVOT_ARM_MOTOR_MAX_OUTPUT, 0);
    this.armMotor.configPeakOutputReverse(-Constants.PIVOT_ARM_MOTOR_MAX_OUTPUT, 0);
    
    this.armMotor.configAllowableClosedloopError(0, 0, 0);

    this.armMotor.config_kF(0, Constants.PIVOT_ARM_MOTOR_kF, 0);
	this.armMotor.config_kP(0, Constants.PIVOT_ARM_MOTOR_kP, 0);
	this.armMotor.config_kI(0, Constants.PIVOT_ARM_MOTOR_kI, 0);
	this.armMotor.config_kD(0, Constants.PIVOT_ARM_MOTOR_kD, 0);

    this.elevatorMotor.configNominalOutputForward(0, 0);
    this.elevatorMotor.configNominalOutputReverse(0, 0);
    this.elevatorMotor.configPeakOutputForward(Constants.TELESCOPING_ARM_MOTOR_MAX_OUTPUT, 0);
    this.elevatorMotor.configPeakOutputReverse(-Constants.TELESCOPING_ARM_MOTOR_MAX_OUTPUT, 0);
    
    this.elevatorMotor.configAllowableClosedloopError(0, 0, 0);

    this.elevatorMotor.config_kF(0, Constants.TELESCOPING_ARM_MOTOR_kF, 0);
	this.elevatorMotor.config_kP(0, Constants.TELESCOPING_ARM_MOTOR_kP, 0);
	this.elevatorMotor.config_kI(0, Constants.TELESCOPING_ARM_MOTOR_kI, 0);
	this.elevatorMotor.config_kD(0, Constants.TELESCOPING_ARM_MOTOR_kD, 0);
  }

  public void startArmMotionMagic(ArmPosition pos) {
    if (this.getArmEncoder() > pos.getPosition()) {
        setArmState(ArmState.Lowering);
    } else if (this.getArmEncoder() < pos.getPosition()) {
        setArmState(ArmState.Raising);
    }

    this.armMotor.set(ControlMode.MotionMagic, pos.getPosition());
  }

  public void checkArmMotionMagicTermination(ArmPosition pos) {
      if (pos == ArmPosition.Starting) {
          armState = ArmState.Stationary;
          // stopArm();
          armPosition = pos;
      } else if (Math.abs(pos.getPosition() - this.getArmEncoder()) <= Constants.CLIMB_ARM_ALLOWED_ERROR) {
          armState = ArmState.Stationary;
          // stopArm();
          armPosition = pos;
      }
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
    if (this.getElevatorEncoder() > pos.getPosition()) {
        setElevatorState(ElevatorState.Lowering);
    } else if (this.getElevatorEncoder() < pos.getPosition()) {
        setElevatorState(ElevatorState.Raising);
    }

    this.elevatorMotor.set(ControlMode.MotionMagic, pos.getPosition());
  }

  public void checkElevatorMotionMagicTermination(ElevatorPosition pos) {
      if (pos == ElevatorPosition.Starting) {
          elevatorState = ElevatorState.Stationary;
          // stopElevator();
          elevatorPosition = pos;
      } else if (Math.abs(pos.getPosition() - this.getElevatorEncoder()) <= Constants.CLIMB_ELEVATOR_ALLOWED_ERROR) {
          elevatorState = ElevatorState.Stationary;
          // stopElevator();
          elevatorPosition = pos;
      }
  }

  public void stopElevator() {
      this.elevatorMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void directElevator(double pow) {
      if (getElevatorState() == ElevatorState.BottomedOut && pow < 0.0) {
          return;
      }
      if (getElevatorState() == ElevatorState.ToppedOut && pow > 0.0) {
          return;
      }
      if (pow > 0.0) {
          setElevatorState(ElevatorState.Raising);
      }
      if (pow < 0.0) {
          setElevatorState(ElevatorState.Lowering);
      }
      if (pow == 0.0) {
          setElevatorState(ElevatorState.Stationary);
      }
      this.elevatorMotor.set(ControlMode.PercentOutput, pow);
  }

  public double getElevatorEncoder() {
    return this.elevatorMotor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Encoder", this.getArmEncoder());
    SmartDashboard.putNumber("Elevator Encoder", this.getElevatorEncoder());
    SmartDashboard.putString("Arm State", this.getArmPosition().toString());
    SmartDashboard.putString("Elevator State", this.getElevatorPosition().toString());
  }
}