package frc.utils.sim_utils;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.RobotBase;

public class RevColorSensorV3Wrapped extends ColorSensorV3 {
  private static ColorSensorV3 colorSensor = null;

  public RevColorSensorV3Wrapped(I2C.Port port) {
    super(port);
    if (RobotBase.isReal()) {
      return;
    }
    if (colorSensor != null) {
      throw new IllegalArgumentException("ColorSensorV3 with port " + port + " already exists");
    }
    colorSensor = this;
  }

  /**
   * This method is used for simulation purposes only. It returns the ColorSensorV3 singleton
   * instance
   *
   * @return The ColorSensorV3 singleton instance
   */
  public static ColorSensorV3 getColorSensor() {
    if (colorSensor == null) {
      throw new IllegalArgumentException("ColorSensorV3 does not exist");
    }
    return colorSensor;
  }
}
