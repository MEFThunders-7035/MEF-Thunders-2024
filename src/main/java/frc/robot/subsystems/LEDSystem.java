package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.LEDConstants;
import frc.utils.BetterLED;

public final class LEDSystem {
  private static final BetterLED strip =
      new BetterLED(LEDConstants.kLedPin, LEDConstants.kLedCount);

  public static void setLEDColorRGB(int r, int g, int b) {
    strip.fill(new Color(r, g, b));
  }
}
