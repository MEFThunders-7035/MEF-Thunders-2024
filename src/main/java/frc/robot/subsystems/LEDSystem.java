package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.LEDConstants;
import frc.utils.BetterLED;
import java.util.function.BooleanSupplier;

public final class LEDSystem {
  private static final BetterLED strip =
      new BetterLED(LEDConstants.kLedPin, LEDConstants.kLedCount);
  private static boolean isRunningACommand = false;

  public static void init() {
    System.out.println("Startup LED System!");
  }

  public static void setLEDColorRGB(int r, int g, int b) {
    strip.fill(new Color(r, g, b));
  }

  public static boolean isRunningACommand() {
    return isRunningACommand;
  }

  public static void setRunningACommand(boolean isRunningACommand) {
    LEDSystem.isRunningACommand = isRunningACommand;
  }

  public static void addTimedCommand(double timeSeconds) {
    isRunningACommand = true;
    new Thread(
            () -> {
              Timer.delay(timeSeconds);
              isRunningACommand = false;
            })
        .start();
  }

  public static void addBooleanSupplierCommand(BooleanSupplier hasEnded) {
    isRunningACommand = true;
    new Thread(
            () -> {
              while (!hasEnded.getAsBoolean()) {
                Timer.delay(0.1);
              }
              isRunningACommand = false;
            })
        .start();
  }
}
