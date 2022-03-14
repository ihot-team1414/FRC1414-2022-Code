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
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.RollingAverage;

public class HoodSubsystem extends SubsystemBase {

  private double target = 0;

  private final Servo leftServo = new Servo(9);
  private final Servo rightServo = new Servo(8);

  private double minAngle = 15; // todo
  private double maxAngle = 80;// todo

  private double previousError = 0.0;
  private double accumulatedError = 0.0;
  private final double height = Constants.TARGET_HEIGHT - Constants.LIMELIGHT_HEIGHT;
  private RollingAverage avg = new RollingAverage();


  public HoodSubsystem() {
    this.previousError = 0.0;
    this.accumulatedError = 0.0;
  }

  public double calculateDistance() {

     return height / Math.tan(Math.toRadians(calculateVisionAngle() + Constants.LIMELIGHT_Y_ANGLE));
  }

  public double calculateEntryAngle(double distance) {
    if(distance > 2 && distance < 4) {
      return Math.toRadians(-50);
    } else if (distance > 4 && distance < 8) {
      return Math.toRadians(-50);
    } else {
      return Math.toRadians(-50);
    }
  }

  public double calculateLaunchAngle(double distance, double entryAngle) {
    double height = Constants.TARGET_HEIGHT;

    return Math.atan((Math.tan(entryAngle)*distance-2*height)/-distance);
  }

  public void visionTargeting() {
    // double distance = calculateDistance();
    // double entryAngle = calculateEntryAngle(distance);
    // double launchAngle = calculateLaunchAngle(distance, entryAngle);

    this.setValue(0.015 * calculateDistance() + 0.22);  
  }

  public double calculateVisionAngle() {
    NetworkTableInstance.getDefault().startClientTeam(1414);
    NetworkTableInstance.getDefault().startDSClient();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");

    avg.add(ty.getDouble(0.0));

    return avg.getAverage();

  }


  public void setAngle(double angle) {
    if (angle > this.maxAngle) {
      angle = this.maxAngle;
    } else if (angle < this.minAngle) {
      angle = this.minAngle;
    }

    target = -0.00702 * angle + 0.77796;

    leftServo.set(target);
    rightServo.set(target);
  }

  public void setValue(double value) {
    target = value;
    leftServo.set(value);
    rightServo.set(value);
  }

  public void resetAngle() {
    leftServo.set(0);
    rightServo.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood Setpoint", target);
    SmartDashboard.putNumber("Distance", calculateDistance());
    SmartDashboard.putNumber("Optimal Angle", Math.toDegrees(calculateLaunchAngle(calculateDistance(), calculateEntryAngle(calculateDistance()))));
  }
}//end of class