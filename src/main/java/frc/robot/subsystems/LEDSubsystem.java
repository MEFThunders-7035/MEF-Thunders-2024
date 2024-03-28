package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.utils.BetterLED;

public class LEDSubsystem extends SubsystemBase {
  BetterLED strip;

  public LEDSubsystem() {
    strip = new BetterLED(LEDConstants.kLedPin, LEDConstants.kLedCount);
  }

  public BetterLED getStrip() {
    return strip;
  }
}
