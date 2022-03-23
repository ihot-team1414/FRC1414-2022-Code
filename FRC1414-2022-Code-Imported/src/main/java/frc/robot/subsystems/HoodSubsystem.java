package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.RollingAverage;

public class HoodSubsystem extends SubsystemBase {
  private double target = 0;

  private final Servo leftServo = new Servo(Constants.HOOD_SERVO_ID_1);
  private final Servo rightServo = new Servo(Constants.HOOD_SERVO_ID_2);

  private final double height = Constants.TARGET_HEIGHT - Constants.LIMELIGHT_HEIGHT;
  private RollingAverage avg = new RollingAverage();

  public HoodSubsystem() {
    SmartDashboard.putNumber("Set Hood", 0.35);
  }

  public double calculateDistance() {
     return height / Math.tan(Math.toRadians(calculateVisionAngle() + Constants.LIMELIGHT_Y_ANGLE));
  }

  double targetValue = 0.35;

  public double calculateVisionAngle() {
    NetworkTableInstance.getDefault().startClientTeam(1414);
    NetworkTableInstance.getDefault().startDSClient();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");

    avg.add(ty.getDouble(0.0));

    return avg.getAverage();
  }

  public void visionTargeting() {
    this.setValue(targetValue);  

    // this.setValue(0.015 * calculateDistance() + 0.22);  
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
    SmartDashboard.putNumber("Hood Target", target);
    targetValue = SmartDashboard.getNumber("Set Hood", 0.35);

    SmartDashboard.putNumber("Goal Distance", calculateDistance());
  }
}