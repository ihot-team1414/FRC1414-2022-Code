package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.util.RollingAverage;
import frc.robot.util.RollingAverage;

public class HoodSubsytem extends SubsystemBase {
  private final CANSparkMax hoodMotor = new CANSparkMax(Constants.HOOD_MOTOR_ID, MotorType.kBrushless);

  private RelativeEncoder hoodEncoder;
  private SparkMaxPIDController pidController;

  private final double maxEncoderTicks = 4.4;
  private double encoderDegree = 0; // TODO

  private double minAngle = 60; // todo
  private double maxAngle = 90;// todo

  private double kP = 5e-5; 
  private double kI = 1e-6;
  private double kD = 0; 
  private double kIz = 0; 
  private double kFF = 0.000156; 
  private double kMaxOutput = 1; 
  private double kMinOutput = -1;

  private double previousError = 0.0;
  private double accumulatedError = 0.0;
  private final double height = Constants.TARGET_HEIGHT - Constants.LIMELIGHT_HEIGHT;
  private RollingAverage avg = new RollingAverage();

  //private RollingAverage avg = new RollingAverage();

  //private final double height = Constants.TARGET_HEIGHT - Constants.LIMELIGHT_HEIGHT;

  public HoodSubsytem() {
    this.hoodEncoder = this.hoodMotor.getEncoder();
    this.pidController = this.hoodMotor.getPIDController();
    this.previousError = 0.0;
    this.accumulatedError = 0.0;
    this.resetEncoder();
    this.hoodMotor.setInverted(false);
    // this.hoodMotor2.setInverted(false);
    this.hoodMotor.setIdleMode(IdleMode.kBrake);
    // this.hoodMotor2.setIdleMode(IdleMode.kBrake);
    this.encoderDegree = (minAngle - maxAngle)/(maxEncoderTicks);

    // this.hoodMotor2.follow(this.hoodMotor);

    // this.hoodEncoder.setInverted(true)

    this.pidController.setP(kP);
    this.pidController.setI(kI);
    this.pidController.setD(kD);
    this.pidController.setIZone(kIz);
    this.pidController.setFF(kFF);
    this.pidController.setOutputRange(kMinOutput, kMaxOutput);
  }


  public double getEncoder() {
    return this.hoodEncoder.getPosition();
  }

  public double calculateDistance() {

     return height / Math.tan(Math.toRadians(calculateVisionAngle() + Constants.LIMELIGHT_Y_ANGLE));
  }

  public double calculateEntryAngle(double distance) {
    if(distance > 2 && distance < 4) {
      return Math.toRadians(-60);
    } else if (distance > 4 && distance < 8) {
      return Math.toRadians(-50);
    } else {
      return Math.toRadians(-80);
    }
  }

  public double calculateLaunchAngle(double distance, double entryAngle) {
    double height = Constants.TARGET_HEIGHT;

    return Math.atan((Math.tan(entryAngle)*distance-2*height)/-distance);
  }

  public void visionTargeting() {
    double distance = calculateDistance();
    double entryAngle = calculateEntryAngle(distance);
    double launchAngle = calculateLaunchAngle(distance, entryAngle);

    this.setAngle(Math.toDegrees(launchAngle));
  }

  public double calculateVisionAngle() {
    NetworkTableInstance.getDefault().startClientTeam(1414);
    NetworkTableInstance.getDefault().startDSClient();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");

    avg.add(ty.getDouble(0.0));

    return avg.getAverage();

  }

  public void moveHood(double throttle) {
    SmartDashboard.putNumber("throttle", throttle);
    if (this.maxAngle > this.getAngle() && this.getAngle() > this.minAngle) {
      this.hoodMotor.set(throttle);
    } else if (this.maxAngle <= this.getAngle()) {
      if (throttle < 0) {
        this.hoodMotor.set(0);
      } else {
        this.hoodMotor.set(throttle);
      }
    } else if (this.minAngle >= this.getAngle()) {
      if (throttle < 0) {
        this.hoodMotor.set(throttle);
      } else {
        this.hoodMotor.set(0);
      }
    }
  }


  public double getAngle() {
    return this.getEncoder() * this.encoderDegree + this.maxAngle;
  }


  public void setAngle(double angle) {
    if (angle > this.maxAngle) {
      angle = this.maxAngle;
    } else if (angle < this.minAngle) {
      angle = this.minAngle;
    }
    double encoderTicks = -(angle - this.maxAngle)*(this.maxEncoderTicks/(this.maxAngle - this.minAngle));
    SmartDashboard.putNumber("Setpoint", encoderTicks);
    this.pidController.setReference(encoderTicks, ControlType.kPosition);
  }

  public void resetAngle() {
    if (this.getAngle() < this.maxAngle) {
      this.setAngle(this.maxAngle);
    } else {
      this.resetEncoder();
    }
  }

  public void resetEncoder() {
    this.hoodEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood Encoder", this.getEncoder());
    SmartDashboard.putNumber("Hood Angle", this.getAngle());
    SmartDashboard.putNumber("Distance", calculateDistance());
    SmartDashboard.putNumber("Optimal Angle", Math.toDegrees(calculateLaunchAngle(calculateDistance(), calculateEntryAngle(calculateDistance()))));
  }






}//end of class