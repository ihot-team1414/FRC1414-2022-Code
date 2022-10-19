package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

  private static ClimbSubsystem instance;

  private final TalonFX telescopingMotor = new TalonFX(Constants.TELESCOPING_ARM_1_MOTOR_ID);
  private final TalonFX telescopingMotor2 = new TalonFX(Constants.TELESCOPING_ARM_2_MOTOR_ID);
  private final TalonFX pivotMotor = new TalonFX(Constants.PIVOT_ARM_1_MOTOR_ID);
  private final TalonFX pivotMotor2 = new TalonFX(Constants.PIVOT_ARM_2_MOTOR_ID);
  private boolean isSpooled = false;
  private boolean isSpooled2 = false;

  private int currentState = 0;
  private boolean stateTriggered = false;

  public static synchronized ClimbSubsystem getInstance() {
    if (instance == null) {
      instance = new ClimbSubsystem();
    }

    return instance;
  }

  public enum PivotPosition {
    Starting(2000), Vertical(70000), Grabbing(62000), Tilting(100000), Lifting(40000);

    private int position;

    PivotPosition(int encPos) {
      position = encPos;
    }

    public int getPosition() {
      return position;
    }
  }

  public enum TelescopePosition {
    Neutral(100), Starting(1414), Intermediate(50000), FirstRung(150000), Extended(190000);

    private int position;

    TelescopePosition(int encPos) {
      position = encPos;
    }

    public int getPosition() {
      return position;
    }
  }

  TelescopePosition[] telescopeStates = {
    TelescopePosition.Neutral,
    TelescopePosition.FirstRung,
    TelescopePosition.Neutral,
    TelescopePosition.Neutral,
    TelescopePosition.Intermediate,
    TelescopePosition.Intermediate,
    TelescopePosition.Extended,
    TelescopePosition.Extended,
    TelescopePosition.Intermediate,
    TelescopePosition.Neutral,
    TelescopePosition.Neutral,
    TelescopePosition.Intermediate,
    TelescopePosition.Intermediate,
    TelescopePosition.Extended,
    TelescopePosition.Extended,
    TelescopePosition.Intermediate,
    TelescopePosition.Neutral,
    TelescopePosition.Neutral,
    TelescopePosition.Intermediate,
    TelescopePosition.Neutral,
  };

  PivotPosition[] pivotStates = {
    PivotPosition.Vertical,
    PivotPosition.Lifting,
    PivotPosition.Lifting,
    PivotPosition.Grabbing,
    PivotPosition.Grabbing,
    PivotPosition.Tilting,
    PivotPosition.Tilting,
    PivotPosition.Vertical,
    PivotPosition.Lifting,
    PivotPosition.Lifting,
    PivotPosition.Grabbing,
    PivotPosition.Grabbing,
    PivotPosition.Tilting,
    PivotPosition.Tilting,
    PivotPosition.Vertical,
    PivotPosition.Lifting,
    PivotPosition.Lifting,
    PivotPosition.Grabbing,
    PivotPosition.Grabbing,
    PivotPosition.Tilting,
  };

  private void configureTelescopingMotor(TalonFX motor) {
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    motor.setNeutralMode(NeutralMode.Brake);

    motor.configPeakOutputForward(Constants.TELESCOPING_ARM_MOTOR_MAX_OUTPUT, 0);
    motor.configPeakOutputReverse(-Constants.TELESCOPING_ARM_MOTOR_MAX_OUTPUT, 0);

    motor.enableVoltageCompensation(true);
    motor.configVoltageCompSaturation(12.0, 30);

    motor.configAllowableClosedloopError(0, Constants.TELESCOPING_ARM_ALLOWED_ERROR, 30);

    motor.selectProfileSlot(0, 0);
    motor.config_kF(0, Constants.TELESCOPING_ARM_MOTOR_kF, 0);
    motor.config_kP(0, Constants.TELESCOPING_ARM_MOTOR_kP, 0);
    motor.config_kI(0, Constants.TELESCOPING_ARM_MOTOR_kI, 0);
    motor.config_kD(0, Constants.TELESCOPING_ARM_MOTOR_kD, 0);

    motor.configMotionCruiseVelocity(Constants.TELESCOPING_ARM_MAX_VEL, 30);
    motor.configMotionAcceleration(Constants.TELESCOPING_ARM_ACCEL, 30);
  }

  private void configurePivotMotor(TalonFX motor) {
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    motor.setNeutralMode(NeutralMode.Brake);
    
    motor.configPeakOutputForward(Constants.PIVOT_ARM_MOTOR_MAX_OUTPUT, 0);
    motor.configPeakOutputReverse(-Constants.PIVOT_ARM_MOTOR_MAX_OUTPUT, 0);

    motor.configAllowableClosedloopError(0, 0, 0);

    motor.selectProfileSlot(0, 0);
    motor.config_kF(0, Constants.PIVOT_ARM_MOTOR_kF, 0);
    motor.config_kP(0, Constants.PIVOT_ARM_MOTOR_kP, 0);
    motor.config_kI(0, Constants.PIVOT_ARM_MOTOR_kI, 0);
    motor.config_kD(0, Constants.PIVOT_ARM_MOTOR_kD, 0);

    motor.configMotionCruiseVelocity(Constants.PIVOT_ARM_MAX_VEL, 30);
    motor.configMotionAcceleration(Constants.PIVOT_ARM_ACCEL, 30);
  }

  private ClimbSubsystem() {
    configureTelescopingMotor(telescopingMotor);
    telescopingMotor.setInverted(true);

    configureTelescopingMotor(telescopingMotor2);

    configurePivotMotor(pivotMotor);
    configurePivotMotor(pivotMotor2);
    pivotMotor2.setInverted(true);
  }

  public void setPivot(PivotPosition pos) {
    pivotMotor.set(ControlMode.MotionMagic, pos.getPosition());
    pivotMotor2.set(ControlMode.MotionMagic, pos.getPosition());
  }

  public void stopPivot() {
    pivotMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void setTelescope(TelescopePosition pos) {
    telescopingMotor.set(ControlMode.Position, pos.getPosition());
    telescopingMotor2.set(ControlMode.Position, pos.getPosition());
  }

  public void stopTelescope() {
    telescopingMotor.set(ControlMode.PercentOutput, 0.0);
    telescopingMotor2.set(ControlMode.PercentOutput, 0.0);
  }

  public void spool() {
    if (!isStalling(telescopingMotor) && !isSpooled) {
      telescopingMotor.set(ControlMode.PercentOutput, -Constants.TELESCOPING_ARM_SPOOL_SPEED);
    } else {
      telescopingMotor.set(ControlMode.PercentOutput, 0);
      telescopingMotor.setSelectedSensorPosition(0);
      isSpooled = true;
    }

    if (!isStalling(telescopingMotor2) && !isSpooled2) {
      telescopingMotor2.set(ControlMode.PercentOutput, -Constants.TELESCOPING_ARM_SPOOL_SPEED);
    } else {
      telescopingMotor2.set(ControlMode.PercentOutput, 0);
      telescopingMotor2.setSelectedSensorPosition(0);
      isSpooled2 = true;
    }
  }

  public void resetSpool() {
    isSpooled = false;
    isSpooled2 = false;
  }

  public boolean isStalling(TalonFX motor) {
    return motor.getSupplyCurrent() >= 5;
  }

  public void activateState() {
    setTelescope(telescopeStates[currentState]);
    setPivot(pivotStates[currentState]);

    stateTriggered = true;
  }

  public void nextState() {
    if (currentState + 1 < telescopeStates.length && stateTriggered) {
      currentState++;
      stateTriggered = false;
    }
  }

  public void previousState() {
    if (currentState - 1 > 0 && stateTriggered) {
      currentState--;
      stateTriggered = false;
    }
  }

  public void resetState() {
    currentState = 0;
  }

  public boolean isPivotAtTarget(PivotPosition pivotPosition) {
    return pivotMotor.getSelectedSensorPosition() < Constants.PIVOT_ARM_ALLOWED_ERROR_FOR_TURRET + pivotPosition.getPosition()
      && pivotMotor.getSelectedSensorPosition() > pivotPosition.getPosition() - Constants.PIVOT_ARM_ALLOWED_ERROR_FOR_TURRET ;
  }

  public boolean isTelescopeAtTarget(TelescopePosition telescopePosition) {
    return telescopingMotor.getSelectedSensorPosition() < Constants.TELESCOPING_ARM_ALLOWED_ERROR + telescopePosition.getPosition()
      && telescopingMotor.getSelectedSensorPosition() > telescopePosition.getPosition() - Constants.TELESCOPING_ARM_ALLOWED_ERROR ;
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Pivot Target", pivotMotor.getClosedLoopTarget());
    SmartDashboard.putNumber("Pivot 1 Position", pivotMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Pivot 2 Position", pivotMotor2.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Pivot 1 Closed Loop Error", pivotMotor.getClosedLoopError());
    // SmartDashboard.putNumber("Pivot 2 Closed Loop Error", pivotMotor2.getClosedLoopError());

    // SmartDashboard.putNumber("Telescope Target", telescopingMotor.getClosedLoopTarget());
    SmartDashboard.putNumber("Telescope 1 Position", telescopingMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Telescope 2 Position", telescopingMotor2.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Telescope 1 Closed Loop Error",
    //     telescopingMotor.getClosedLoopError());
    // SmartDashboard.putNumber("Telescope 2 Closed Loop Error",
    //     telescopingMotor2.getClosedLoopError());
  }
}