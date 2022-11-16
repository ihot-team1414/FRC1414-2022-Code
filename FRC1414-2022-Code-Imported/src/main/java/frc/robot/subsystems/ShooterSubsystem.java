package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private static ShooterSubsystem instance;

  //Create object reference to both shooter motors used
  private final TalonFX shooterMotor1 = new TalonFX(Constants.SHOOTER_ID_1);
  private final TalonFX shooterMotor2 = new TalonFX(Constants.SHOOTER_ID_2);

  private double speed = 0;

  //Return single instance of IndexerSubsystem Class (Singleton)
  public static synchronized ShooterSubsystem getInstance() {
    if (instance == null) {
      instance = new ShooterSubsystem();
    }

    return instance;
  }

  //Constructor values are applied upon creation / reference of ShooterSubsystem object/class
  public ShooterSubsystem() {
    
    //Sets configurations for motors
    shooterMotor1.selectProfileSlot(0, 0);
    shooterMotor1.config_kF(0, Constants.SHOOTER_kF, 0);
    shooterMotor1.config_kP(0, Constants.SHOOTER_kP, 0);
    shooterMotor1.config_kI(0, Constants.SHOOTER_kI, 0);
    shooterMotor1.config_kD(0, Constants.SHOOTER_kD, 0);
    shooterMotor1.enableVoltageCompensation(true);
    shooterMotor1.configVoltageCompSaturation(12.0, 30);
    shooterMotor1.configPeakOutputForward(Constants.SHOOTER_MAX_OUTPUT);
    shooterMotor1.setInverted(true);
    shooterMotor1.setNeutralMode(NeutralMode.Coast);

    //Shooter motor two follows the control mode and output of shooter motor 1
    shooterMotor2.follow(shooterMotor1);
    shooterMotor2.setNeutralMode(NeutralMode.Coast);
    shooterMotor2.enableVoltageCompensation(true);
    shooterMotor2.configVoltageCompSaturation(12.0, 30);

    if (Constants.MANUAL_SPEED_AND_ANGLE) {
      SmartDashboard.putNumber("Dashboard Shooter Target", 9000);
    }
  }

  //Shooting method
  //@param speed acts as input for the speed for shooter motor one
  public void shoot(double speed) {
    // 21700 is max theoretical speed for shooter.
    this.speed = speed;

    //IGNORE
    //THERMOSTAT
    // if (shooterMotor1.getSelectedSensorVelocity() >= speed) {
    //   shooterMotor1.set(ControlMode.PercentOutput, 0);
    // } else {
    //   shooterMotor1.set(ControlMode.PercentOutput, 0.25);
    // }

    //Takes the @param speed as input for the speed of the motor
    shooterMotor1.set(ControlMode.Velocity, speed);
  }

  //Layup method
  public void layup() {
    //Set speed to predetermined constant
    speed = Constants.SHOOTER_LAYUP_SPEED;
    //Set motor speed to the constant
    shooterMotor1.set(ControlMode.Velocity, speed);
  }

  //Check whether or not the shooter is within alllowed error
  public boolean isWithinAllowedError() {
    double allowedError = Constants.SHOOTER_ALLOWED_ERROR;
    if (speed >= 7000) {
      allowedError = Constants.SHOOTER_ALLOWED_ERROR * (1 + ((speed - 7000) / 4000));
    }
    return Math.abs(shooterMotor1.getSelectedSensorVelocity() - speed) < allowedError;
  }

  //Get the shooters speed
  public double getShooterSpeed() {
    return shooterMotor1.getSelectedSensorVelocity();
  }

  //Set the shooter motor to 5000 (velocity mode) to eject the ball
  public void eject() {
    shooterMotor1.set(ControlMode.Velocity, 5000);
  }

  //Set the output of the motor to 0 / stop it
  public void stop() {
    shooterMotor1.set(ControlMode.PercentOutput, 0.0);
    speed = 0;
  }

  //Periodically (20 ms) call the following
  @Override
  public void periodic() {
    //On the smart dashboard application, update the following
    SmartDashboard.putNumber("Shooter Target", shooterMotor1.getClosedLoopTarget());
    SmartDashboard.putNumber("Shooter Velocity", shooterMotor1.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("Shooter Closed Loop Error", shooterMotor1.getClosedLoopError());
  }
}