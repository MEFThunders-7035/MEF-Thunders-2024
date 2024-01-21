package frc.utils;

public class ExtraFunctions {
  /**
   * This is used to map a value from one range to another. Taken from:
   * https://www.arduino.cc/reference/en/language/functions/math/map/
   *
   * @param x the value to map
   * @param in_min the minimum value of the input range
   * @param in_max the maximum value of the input range
   * @param out_min the minimum value of the output range
   * @param out_max the maximum value of the output range
   * @return the mapped value
   */
  public static double mapValue(
      double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
}
