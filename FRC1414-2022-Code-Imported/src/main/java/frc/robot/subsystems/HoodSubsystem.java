package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.Limelight;
import frc.util.ShooterData;

public class HoodSubsystem extends SubsystemBase {
  private final Servo servo1 = new Servo(Constants.HOOD_SERVO_ID_1);
  private final Servo servo2 = new Servo(Constants.HOOD_SERVO_ID_2);

  private double dashboardTarget = 0;

  public HoodSubsystem() {
    if (Constants.MANUAL_SPEED_AND_ANGLE) {
      SmartDashboard.putNumber("Dashboard Hood Target", Constants.HOOD_MIN);
    }
  }

  public void visionTargeting() {
    double ty = Limelight.getInstance().getDeltaY();

    double target = ShooterData.getInstance().getHoodAngle(ty);

    if (Constants.MANUAL_SPEED_AND_ANGLE) {
      set(dashboardTarget);
    } else if (Limelight.getInstance().detectsTarget()) {
      SmartDashboard.putNumber("Hood Target", target);
      set(target);
    }
  }

  public void set(double value) {
    if (value < Constants.HOOD_MIN) {
      value = Constants.HOOD_MIN;
    } else if (value > Constants.HOOD_MAX) {
      value = Constants.HOOD_MAX;
    }

    servo1.set(value);
    servo2.set(value);
  }

  public void home() {
    set(Constants.HOOD_MIN);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Goal Y", Limelight.getInstance().getDeltaY());
    dashboardTarget = SmartDashboard.getNumber("Dashboard Hood Target", 0.0);
  }
}