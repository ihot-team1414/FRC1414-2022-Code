package frc.robot.subsystems;

import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.Limelight;

public class HoodSubsystem extends SubsystemBase {
  private double target = 0;


  private final Servo servo1 = new Servo(Constants.HOOD_SERVO_ID_1);
  private final Servo servo2 = new Servo(Constants.HOOD_SERVO_ID_2);

  public HoodSubsystem() {
  }


  public void visionTargeting() {
    double ty = Limelight.getInstance().getDeltaY();

    

    double setpoint = Limelight.getInstance().detectsTarget() ? 0.254*Math.pow(Math.E, -0.0164881818*ty) : target;

    SmartDashboard.putNumber("Hood Target", setpoint);

    set(setpoint);
  }

  public void set(double value) {
    if (value < 0.22) {
      value = 0.22;
    } else if (value > 0.5) {
      value = 0.5;
    }

    target = value;

    servo1.set(value);
    servo2.set(value);
  }

  public void home() {
    set(0.35);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Goal Y", Limelight.getInstance().getDeltaY());
  }
}