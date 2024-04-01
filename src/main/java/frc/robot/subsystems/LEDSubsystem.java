package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.utils.BetterLED;

public class LEDSubsystem extends SubsystemBase {
  BetterLED strip;
  Color lastSetColor = new Color();
  Color lastSetBlinkingColor = new Color();
  boolean isBlinkingRed = false;

  public LEDSubsystem() {
    strip = new BetterLED(LEDConstants.kLedPin, LEDConstants.kLedCount);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("LED Currently Running", strip.getCurrentCommandName());
  }

  public void fill(Color color) {
    if (lastSetColor.equals(color)) {
      return;
    }
    strip.fillColor(color);
    lastSetColor = color;
    isBlinkingRed = false;
  }

  public void fill(Color color, int count) {
    fill(color, count, false);
  }

  public void fill(Color color, int count, boolean force) {
    if (lastSetColor.equals(color) && !force) {
      return;
    }
    strip.fillColor(color, count);
    lastSetColor = color;
  }

  public Command getRedBlinkCommand() {
    return this.run(this::blinkRed)
        .finallyDo(
            () -> {
              strip.removeFromLoop();
              lastSetBlinkingColor = new Color();
            });
  }

  public void setRed() {
    fill(Color.kRed);
  }

  public void setGreen() {
    fill(Color.kGreen);
  }

  public void setStatusColor(boolean status) {
    if (status) {
      setGreen();
    } else {
      setRed();
    }
  }

  public void blink(Color color) {
    if (lastSetBlinkingColor.equals(color)) {
      return;
    }
    lastSetColor = new Color(); // Reset back to 0
    lastSetBlinkingColor = color;
    strip.blink(color);
  }

  public void blinkRed() {
    if (isBlinkingRed) {
      return;
    }
    blink(new Color(255, 0, 0));
  }

  public void fillPercentageWithColor(double percentage, Color color) {
    fill(color, (int) (strip.getLedCount() * percentage), true);
  }

  public BetterLED getStrip() {
    return strip;
  }
}