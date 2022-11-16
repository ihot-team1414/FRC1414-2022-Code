package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {
  private static IndexerSubsystem instance;

  //Creates object reference for every single index-related motor
  //@param Constants.XYZ_MOTOR_ID is stored in the Constants file
  //@param Constants.XYZ_MOTOR_ID specifies the ID of the port on the RoboRIO that the motor is connected to
  private final TalonSRX loaderFrontMotor = new TalonSRX(Constants.LOADER_FRONT_MOTOR_ID);
  private final TalonSRX loaderBackMotor = new TalonSRX(Constants.LOADER_BACK_MOTOR_ID);
  private final TalonSRX funnelMotor = new TalonSRX(Constants.FUNNEL_MOTOR_ID);
  
  //Return single instance of IndexerSubsystem Class (Singleton)
  public static synchronized IndexerSubsystem getInstance() {
    
    //If instance doesn't already exist, create it
    if (instance == null) {
      instance = new IndexerSubsystem();
    }

    //Return the instance
    return instance;
  }

  //Everything within the constructor is added upon the creation of an IndexerSubsystem reference  
  private IndexerSubsystem() {
    
    //Motors Neutral Mode is set
    loaderFrontMotor.setNeutralMode(NeutralMode.Brake);
    loaderBackMotor.setNeutralMode(NeutralMode.Brake);
    funnelMotor.setNeutralMode(NeutralMode.Coast);

    //Both referenced motors are inverted
    loaderFrontMotor.setInverted(true);    
    loaderBackMotor.setInverted(true);
  }

  //Method to move ball through the indexer
  public void loadShooter() {

    //If the shooter speed is greater than the minimum speed required by the shooter (specified in Constants)
    if (ShooterSubsystem.getInstance().getShooterSpeed() > Constants.SHOOTER_MIN_LOAD_SPEED) {

      //Allow motors to take in balls
      loaderBackMotor.set(ControlMode.PercentOutput, Constants.LOADING_SPEED);
      loaderFrontMotor.set(ControlMode.PercentOutput, Constants.LOADING_SPEED);
    }
  }

  //Method to hold the balls
  public void holdBall() {
    loaderBackMotor.set(ControlMode.PercentOutput, -Constants.HOLDING_SPEED);
    loaderFrontMotor.set(ControlMode.PercentOutput, Constants.HOLDING_SPEED);  
  }

  //Set all output values to 0% (stop the motors from moving)
  public void stop() {
    loaderBackMotor.set(ControlMode.PercentOutput, 0.0);
    loaderFrontMotor.set(ControlMode.PercentOutput, 0.0);
    funnelMotor.set(ControlMode.PercentOutput, 0.0);
  }
  
  //TO-DO
  public void funnel() {
    funnelMotor.set(ControlMode.PercentOutput, Constants.FUNNEL_SPEED);
  }

  //Reverse indexer direction
  public void reverse() {
    funnelMotor.set(ControlMode.PercentOutput, -Constants.OUTTAKE_FUNNEL_SPEED);
    loaderBackMotor.set(ControlMode.PercentOutput, -Constants.LOADING_SPEED);
    loaderFrontMotor.set(ControlMode.PercentOutput, -Constants.LOADING_SPEED);  
  }

  //Hold Ball Method
  public void holdBalls() {
    funnel();
    holdBall();
  }

  //Load balls into shooter
  public void load() {
    funnel();
    loadShooter();
  }
}