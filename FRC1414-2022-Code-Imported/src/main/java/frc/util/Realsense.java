package frc.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Realsense {
    
    private static Realsense instance = null;
    private double zeroedX = 0;
    private double zeroedY = 0;
    private double zeroedYaw = 0;

    private double initialX = 0;
    private double initialY = 0;
    private double initialYaw = 0;

    private NetworkTableEntry x = null;
    private NetworkTableEntry y = null;
    private NetworkTableEntry yaw = null;

    private Realsense() {
        NetworkTableInstance.getDefault().startClientTeam(1414);
        NetworkTableInstance.getDefault().startDSClient();
        NetworkTable realsenseData = NetworkTableInstance.getDefault().getTable("realsense");

        x = realsenseData.getEntry("x");
        y = realsenseData.getEntry("y");
        yaw = realsenseData.getEntry("yaw");
    }

    public void zero() {
        if (x != null && y != null && yaw != null) {
            zeroedX = x.getDouble(0.0);
            zeroedY = y.getDouble(0.0);
            zeroedYaw = yaw.getDouble(0.0);
        }
    }

    public void setInitial(double x, double y, double yaw) {
        initialX = x;
        initialY = y;
        initialYaw = yaw;
    }

    public double getX() {
        return x.getDouble(0.0) - zeroedX + initialX;
    }

    public double getY() {
        return y.getDouble(0.0) - zeroedY + initialY;
    }

    public double getYaw() {
        return yaw.getDouble(0.0) - zeroedYaw + initialYaw;
    }

    public static Realsense getInstance() {

        if (instance == null) {
            instance = new Realsense();
        }

        return instance;
    }
}
