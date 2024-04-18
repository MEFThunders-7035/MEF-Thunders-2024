package utils;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;

public class LEDTestUtils {
  public static void checkForColorInAll(
      LEDSubsystem ledSubsystem, Color colorItShouldBe, String message) {
    for (int i = 0; i < ledSubsystem.getLedCount(); i++) {
      assertEquals(colorItShouldBe, getColorAtIndex(ledSubsystem, i), message + " at index: " + i);
    }
  }

  public static Color getColorAtIndex(LEDSubsystem ledSubsystem, int index) {
    return ledSubsystem.getStrip().getBuffer().getLED(index);
  }
}
