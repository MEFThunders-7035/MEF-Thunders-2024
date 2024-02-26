package frc.utils.sim_utils;

import com.revrobotics.CANSparkMax;
import java.util.HashMap;
import java.util.Map;

public class SparkMAXSimAddon {
  private static Map<Integer, CANSparkMax> sparkMaxes = new HashMap<>();

  public static void addSparkMAX(CANSparkMax sparkMAX) {
    if (sparkMaxes.containsKey(sparkMAX.getDeviceId())) {
      throw new IllegalArgumentException(
          "SparkMAX with deviceID " + sparkMAX.getDeviceId() + " already exists");
    }
    sparkMaxes.put(sparkMAX.getDeviceId(), sparkMAX);
  }

  public static CANSparkMax getSparkMAX(int deviceID) {
    if (!sparkMaxes.containsKey(deviceID)) {
      throw new IllegalArgumentException("SparkMAX with deviceID " + deviceID + " does not exist");
    }
    return sparkMaxes.get(deviceID);
  }
}
