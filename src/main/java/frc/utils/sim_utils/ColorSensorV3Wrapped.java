package frc.utils.sim_utils;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.RobotBase;

public class ColorSensorV3Wrapped extends ColorSensorV3 implements AutoCloseable {
  private static ColorSensorV3 colorSensor = null;
  private static final int[] rgbd = new int[4]; // Red, Green, Blue, and Distance

  public ColorSensorV3Wrapped(I2C.Port port) {
    super(port);
    if (RobotBase.isReal()) {
      return;
    }
    // Do not check if colorSensor is null because if you do the unit test crashes for some reason.
    colorSensor = this;
  }

  @Override
  public void close() {
    if (RobotBase.isReal()) {
      return;
    }
    colorSensor = null;
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

  public static void setRGBD(int red, int green, int blue, int distance) {
    if (RobotBase.isReal()) {
      return;
    }
    rgbd[0] = red;
    rgbd[1] = green;
    rgbd[2] = blue;
    rgbd[3] = distance;
  }

  @Override
  public int getRed() {
    // we check if getRed() is 0 because the color sensor returns 0 when its not being "simulated"
    // as this is only used for unit tests.
    if (RobotBase.isReal() || colorSensor == null || colorSensor.getRed() != 0) {
      return super.getRed();
    }
    return rgbd[0];
  }

  @Override
  public int getGreen() {
    if (RobotBase.isReal() || colorSensor == null || colorSensor.getGreen() != 0) {
      return super.getGreen();
    }
    return rgbd[1];
  }

  @Override
  public int getBlue() {
    if (RobotBase.isReal() || colorSensor == null || colorSensor.getBlue() != 0) {
      return super.getBlue();
    }
    return rgbd[2];
  }

  @Override
  public int getProximity() {
    if (RobotBase.isReal() || colorSensor == null || colorSensor.getProximity() != 0) {
      return super.getProximity();
    }
    return rgbd[3];
  }
}
