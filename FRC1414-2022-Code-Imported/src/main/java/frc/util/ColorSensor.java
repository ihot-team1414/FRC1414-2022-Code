package frc.util;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

public class ColorSensor {
  private final ColorMatch colorMatcher = new ColorMatch();

  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  private static ColorSensor instance = null;

  private ColorSensor() {
    colorMatcher.addColorMatch(Constants.BLUE_TARGET);  
    colorMatcher.addColorMatch(Constants.RED_TARGET);  
  }

  public static ColorSensor getInstance() {
    if (instance == null) {
      instance = new ColorSensor();
    }

    return instance;
  }

  public boolean isCorrectColor() {
    Alliance allianceColor = DriverStation.getAlliance();

    Color detectedColor = colorSensor.getColor();

    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    return match.color == Constants.RED_TARGET && allianceColor == Alliance.Red || match.color == Constants.BLUE_TARGET && allianceColor == Alliance.Blue;
  }
}
