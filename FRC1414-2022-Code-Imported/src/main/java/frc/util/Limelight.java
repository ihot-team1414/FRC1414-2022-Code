package frc.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class Limelight {

  private static Limelight instance = null;
  private NetworkTableEntry ty = null;
  private NetworkTableEntry tx = null;

  private RollingAverage avg = new RollingAverage();

  private final double deltaHeight = Constants.TARGET_HEIGHT - Constants.LIMELIGHT_HEIGHT;

  private Limelight() {
    NetworkTableInstance.getDefault().startClientTeam(1414);
    NetworkTableInstance.getDefault().startDSClient();

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    ty = table.getEntry("ty");
    tx = table.getEntry("tx");
  }

  public static Limelight getInstance() {
    if (instance == null) {
      instance = new Limelight();
    }

    return instance;
  }

  public double getDeltaX() {
    if (tx != null) {
      return tx.getDouble(0.0);
    }

    return 0.0;
  }

  public double calculateVisionAngle() {
    if (ty != null) {
      avg.add(ty.getDouble(0.0));
    } else {
      avg.add(0.0);
    }

    return avg.getAverage();
  }

  public double calculateDistance() {
    double distance = deltaHeight / Math.tan(
        Math.toRadians(calculateVisionAngle() + Constants.LIMELIGHT_Y_ANGLE)
    );

    return distance;
  }
}
