package frc.util;

import java.util.HashMap;
import java.util.Map;

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

  private static Color previousColor;

  private ColorSensor() {
    colorMatcher.addColorMatch(Constants.BLUE_TARGET);  
    colorMatcher.addColorMatch(Constants.RED_TARGET);

    previousColor = Color.kBlack;
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

    Map<Color, DriverStation.Alliance> colorCombos = new HashMap<>();
    colorCombos.put(Constants.RED_TARGET, Alliance.Red);
    colorCombos.put(Constants.BLUE_TARGET, Alliance.Blue);

    boolean isCurrentMatch = match.color != null && allianceColor == colorCombos.get(match.color);
    boolean isPreviousColor = allianceColor == colorCombos.get(previousColor);

    if (match.color != null) {
      previousColor = match.color;
    }

    return isCurrentMatch || isPreviousColor;
  }
}
