package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.utils.BetterLED;

public class LEDSubsystem extends SubsystemBase implements AutoCloseable {
  BetterLED strip;
  Color lastSetColor = new Color();
  Color lastSetBlinkingColor = new Color();
  Timer loadingTimer;

  public LEDSubsystem() {
    strip = new BetterLED(LEDConstants.kLedPin, LEDConstants.kLedCount);
    loadingTimer = new Timer();
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("LED Currently Running", strip.getCurrentCommandName());
  }

  @Override
  public void close() {
    strip.close();
  }

  public int getLedCount() {
    return strip.getLedCount();
  }

  public void fill(Color color) {
    if (lastSetColor.equals(color)) {
      return;
    }
    strip.fillColor(color);
    lastSetColor = color;
    lastSetBlinkingColor = Color.kBlack;
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
    lastSetBlinkingColor = Color.kBlack;
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

  private void blink(Color color, double delaySeconds) {
    if (lastSetBlinkingColor.equals(color)) {
      return;
    }
    lastSetColor = new Color(); // Reset back to 0
    lastSetBlinkingColor = color;
    strip.blink(color, delaySeconds);
  }

  private void setBlinkColor(Color color) {
    blink(color, 0.2);
  }

  /**
   * Fills the led's until the given percentage.
   *
   * @param percentage a value between 0 and 1
   * @param color the color to fill with
   */
  public void fillPercentageWithColor(double percentage, Color color) {
    percentage = MathUtil.clamp(percentage, 0.0, 1.0);
    fill(color, (int) (strip.getLedCount() * percentage), true);
  }

  public BetterLED getStrip() {
    return strip;
  }

  public Command idle(BooleanSupplier status) {
    return run(() -> {
      if (DriverStation.isEnabled()) {
        setStatusColor(status.getAsBoolean());
      } else {
        fill(Color.kOrangeRed);
      }
    });
  }

  public Command blinkColor(Color color) {
    return run(() -> setBlinkColor(color))
        .finallyDo(
            () -> {
              strip.removeFromLoop();
              lastSetBlinkingColor = new Color();
            });
  }

  public Command blinkRed() {
    return blinkColor(new Color(255, 0, 0))
        .finallyDo(
            () -> {
              strip.removeFromLoop();
              lastSetBlinkingColor = new Color();
            });
  }

  public Command loadingAnimation(double seconds) {
    Color loadingColor = new Color(0, 200, 255);

    return run(() -> fillPercentageWithColor(loadingTimer.get() / seconds, loadingColor))
      .until(() -> loadingTimer.get() > seconds)
      .beforeStarting(() -> loadingTimer.reset());
  }
}
