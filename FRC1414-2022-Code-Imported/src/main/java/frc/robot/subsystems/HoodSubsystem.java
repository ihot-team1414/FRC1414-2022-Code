package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.Limelight;

public class HoodSubsystem extends SubsystemBase {
  private double target = 0;
  double dashboardTarget = 0.35;

  private final Servo servo1 = new Servo(Constants.HOOD_SERVO_ID_1);
  private final Servo servo2 = new Servo(Constants.HOOD_SERVO_ID_2);

  public HoodSubsystem() {
    SmartDashboard.putNumber("Dashboard Hood Target", 0.35);
  }

  public void visionTargeting() {
    double ty = Limelight.getInstance().getDeltaY();

    double setpoint = 0.28*Math.pow(ty, 2) + 0.002833005*ty + 3.7674092E-4;

    set(setpoint);
  }

  public void set(double value) {
    if (value < 0.25) {
      value = 0.25;
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
    SmartDashboard.putNumber("Hood Target", target);

    SmartDashboard.putNumber("Goal Y", Limelight.getInstance().getDeltaY());
  }
}