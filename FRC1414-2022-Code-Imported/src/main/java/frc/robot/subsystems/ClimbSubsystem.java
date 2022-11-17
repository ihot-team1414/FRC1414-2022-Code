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

  //Create object reference for climbing hardware
  private final TalonFX telescopingMotor = new TalonFX(Constants.TELESCOPING_ARM_1_MOTOR_ID);
  private final TalonFX telescopingMotor2 = new TalonFX(Constants.TELESCOPING_ARM_2_MOTOR_ID);
  private final TalonFX pivotMotor = new TalonFX(Constants.PIVOT_ARM_1_MOTOR_ID);
  private final TalonFX pivotMotor2 = new TalonFX(Constants.PIVOT_ARM_2_MOTOR_ID);
  private boolean isSpooled = false;
  private boolean isSpooled2 = false;

  //index for the current state of climbing
  private int currentState = 0;
  //Check whether a state had been triggered
  private boolean stateTriggered = false;

  //Return single instance of IndexerSubsystem Class (Singleton)
  public static synchronized ClimbSubsystem getInstance() {
    if (instance == null) {
      instance = new ClimbSubsystem();
    }

    return instance;
  }

//Different pivoting positions
//Uses the below * as format
  public enum PivotPosition {
    Starting(2000), Vertical(70000), Grabbing(62000), Tilting(100000), Lifting(40000);

    private int position;

    // the format the positions follow (every "PivotPosition" is joined by @param encPos)
    // example: Starting(2000) is a PivotPosition with an encoder position value of 2000
    // Starting (alongside the encoder position value of 2000) is one of the values that PivotPosition can take 
    PivotPosition(int encPos) {
      position = encPos;
    }

    public int getPosition() {
      return position;
    }
  }

  public enum TelescopePosition {
    //Different telescoping positions
    //Uses the below * as format
    Neutral(100), Starting(1414), Intermediate(50000), FirstRung(150000), Extended(190000);

    private int position;

    // the format the positions follow (every "TelescopePosition" is joined by @param encPos)
    // example: Starting(1414) is a TelescopePosition with an encoder position value of 1414
    // Starting (alongside the encoder position value of 1414) is one of the values that TelescopePosition can take 
    TelescopePosition(int encPos) {
      position = encPos;
    }

    public int getPosition() {
      return position;
    }
  }

  //Array of all possible telescoping states, in order of execution
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

  //Array of all possible pivoting states, in order of execution
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

  //Configure telescoping motors
  //@param motor takes TalonFX motors as input for configuration
  private void configureTelescopingMotor(TalonFX motor) {

    //TalonFX -- Connecting the integrated encoder to the motor
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    
    //mode of operation whilst neutral (not moving -- brake)
    motor.setNeutralMode(NeutralMode.Brake);

    //Configure the peak outputs for both direction of the motor
    motor.configPeakOutputForward(Constants.TELESCOPING_ARM_MOTOR_MAX_OUTPUT, 0);
    motor.configPeakOutputReverse(-Constants.TELESCOPING_ARM_MOTOR_MAX_OUTPUT, 0);

    //Robot draws constant level of voltage, rather than proportionally scaling with voltage available
    //If no more voltage is available, motors will stop functioning
    motor.enableVoltageCompensation(true);

    //TODO
    motor.configVoltageCompSaturation(12.0, 30);

    //TODO
    motor.configAllowableClosedloopError(0, Constants.TELESCOPING_ARM_ALLOWED_ERROR, 30);

    //TODO
    motor.selectProfileSlot(0, 0);
    motor.config_kF(0, Constants.TELESCOPING_ARM_MOTOR_kF, 0);
    motor.config_kP(0, Constants.TELESCOPING_ARM_MOTOR_kP, 0);
    motor.config_kI(0, Constants.TELESCOPING_ARM_MOTOR_kI, 0);
    motor.config_kD(0, Constants.TELESCOPING_ARM_MOTOR_kD, 0);

    //TODO
    motor.configMotionCruiseVelocity(Constants.TELESCOPING_ARM_MAX_VEL, 30);
    motor.configMotionAcceleration(Constants.TELESCOPING_ARM_ACCEL, 30);
  }

  //Configure pivoting motors 
  //@param motor takes TalonFX motors as input for configuration
  private void configurePivotMotor(TalonFX motor) {
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    motor.setNeutralMode(NeutralMode.Brake);
    
  //Configure the peak outputs for both direction of the motor
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

  //Constructor values are applied upon creation / reference of ShooterSubsystem object/class
  private ClimbSubsystem() {
    //Configures the telescopingMotor
    configureTelescopingMotor(telescopingMotor);
    //Inverts motor
    telescopingMotor.setInverted(true);
    //Configures the second telescopingMotor
    configureTelescopingMotor(telescopingMotor2);
    //Configure first and second pivot motor
    configurePivotMotor(pivotMotor);
    configurePivotMotor(pivotMotor2);
    //Set pivot motor two to be inverted
    pivotMotor2.setInverted(true);
  }

  //Set the position of the pivot motors based on the inputed PivotPosition
  public void setPivot(PivotPosition pos) {
    pivotMotor.set(ControlMode.MotionMagic, pos.getPosition());
    pivotMotor2.set(ControlMode.MotionMagic, pos.getPosition());
  }

  //Stop the first pivot motor from moving
  public void stopPivot() {
    pivotMotor.set(ControlMode.PercentOutput, 0.0);
  }

  //Set the position of the telescoping motors based on the inputed TelescopePosition
  public void setTelescope(TelescopePosition pos) {
    telescopingMotor.set(ControlMode.Position, pos.getPosition());
    telescopingMotor2.set(ControlMode.Position, pos.getPosition());
  }

  //Stop the telescoping motors from moving
  public void stopTelescope() {
    telescopingMotor.set(ControlMode.PercentOutput, 0.0);
    telescopingMotor2.set(ControlMode.PercentOutput, 0.0);
  }

  //TODO
  public void spool() {
    if (!isStalling(telescopingMotor) && !isSpooled) {
      telescopingMotor.set(ControlMode.PercentOutput, -Constants.TELESCOPING_ARM_SPOOL_SPEED);
    } else {
      telescopingMotor.set(ControlMode.PercentOutput, 0);
      telescopingMotor.setSelectedSensorPosition(0);
      isSpooled = true;
    }

    //TODO
    if (!isStalling(telescopingMotor2) && !isSpooled2) {
      telescopingMotor2.set(ControlMode.PercentOutput, -Constants.TELESCOPING_ARM_SPOOL_SPEED);
    } else {
      telescopingMotor2.set(ControlMode.PercentOutput, 0);
      telescopingMotor2.setSelectedSensorPosition(0);
      isSpooled2 = true;
    }
  }

  //Set spooled to false to reset spools later
  public void resetSpool() {
    isSpooled = false;
    isSpooled2 = false;
  }

  //If the input current, in amps, is greater than 5, the motor is stalling
  public boolean isStalling(TalonFX motor) {
    return motor.getSupplyCurrent() >= 5;
  }

  //Activate the state at the currentState index
  public void activateState() {
    setTelescope(telescopeStates[currentState]);
    setPivot(pivotStates[currentState]);

    //Selected state has been triggered
    stateTriggered = true;
  }

  //Increment to next state in the array using the index
  public void nextState() {
    //If the state you are moving to is within the length of the array and the latest state has been triggered
    if (currentState + 1 < telescopeStates.length && stateTriggered) {
      
      //Move to last state
      currentState++;
      //Set state triggered to false
      stateTriggered = false;
    }
  }

  //Increment to previous state in the array using the index
  public void previousState() {
    //If the previous state is within the length of the array and the latest state has been triggered
    if (currentState - 1 > 0 && stateTriggered) {

      //Move to the previous state
      currentState--;
      //Set state triggered to false
      stateTriggered = false;
    }
  }

  //Go back to the first state
  public void resetState() {
    currentState = 0;
  }

  //Check if pivot is at its target position
  public boolean isPivotAtTarget(PivotPosition pivotPosition) {

    //If the pivot motor encoder states that the pivot motor is between the error boundaries of the position, return true
    return pivotMotor.getSelectedSensorPosition() < Constants.PIVOT_ARM_ALLOWED_ERROR_FOR_TURRET + pivotPosition.getPosition()
      && pivotMotor.getSelectedSensorPosition() > pivotPosition.getPosition() - Constants.PIVOT_ARM_ALLOWED_ERROR_FOR_TURRET ;
  }

  //If the telescoping motor encoder states that the telescoping motor is between the error boundaries of the position, return true
  public boolean isTelescopeAtTarget(TelescopePosition telescopePosition) {
    return telescopingMotor.getSelectedSensorPosition() < Constants.TELESCOPING_ARM_ALLOWED_ERROR + telescopePosition.getPosition()
      && telescopingMotor.getSelectedSensorPosition() > telescopePosition.getPosition() - Constants.TELESCOPING_ARM_ALLOWED_ERROR ;
  }

  //Periodically (20 ms) update the following values for smart dashboard
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