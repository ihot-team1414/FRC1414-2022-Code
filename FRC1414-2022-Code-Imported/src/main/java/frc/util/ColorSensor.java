package frc.util;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

public class ColorSensor {
  private final ColorMatch colorMatcher = new ColorMatch();

  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  public ColorSensor() {
    colorMatcher.addColorMatch(Constants.BLUE_TARGET);  
    colorMatcher.addColorMatch(Constants.RED_TARGET);  
  }

  public boolean checkColorIsRed() {
    Color detectedColor = colorSensor.getColor();

    boolean isRed;

    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    if (match.color == Constants.RED_TARGET) {
      isRed = true;
    } else {
      isRed = false;
    }

    return isRed;
  }

  public boolean checkColor() {
    NetworkTableInstance.getDefault().startClientTeam(1414);
    NetworkTableInstance.getDefault().startDSClient();

    NetworkTable table = NetworkTableInstance.getDefault().getTable("FMSInfo");

    return false;

  }
}
