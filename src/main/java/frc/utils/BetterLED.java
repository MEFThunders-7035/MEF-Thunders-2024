package frc.utils;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import java.util.ArrayList;

public class BetterLED extends AddressableLED {
  public enum AnimationType { // Is already static.
    LEFT_TO_RIGHT,
    RIGHT_TO_LEFT
  }

  private AddressableLEDBuffer mainBuffer;
  private int ledCount;
  private int rainbowFirstPixelHue = 0;
  private int currentAnimationIndex = 0;

  // The call list will have all animation calls in it,
  // This will run like a scheduler, just on a smaller scale.
  // it will probably have only a single function, but is a an Arraylist just in case.
  private ArrayList<Runnable> callList;

  // The Thread will handle calling the callList and all other requirements for animations.
  private Thread ledUpdateThread;

  public BetterLED(int port, int ledCount) {
    super(port);
    mainBuffer = new AddressableLEDBuffer(ledCount);
    this.setLength(mainBuffer.getLength());
    this.start();
    callList = new ArrayList<>(3); // 3 should be enough on most cases.
    // Create an update thread that handles animations
    ledUpdateThread =
        new Thread(
            () -> {
              for (; ; ) { // Infinity call periodic,
                periodic();
                // Maybe add a delay?
                // or should the delay be handled by the respective functions?
              }
            },
            "LED Control Thread");
    ledUpdateThread.start();
    breathe(new Color(0, 200, 255));
  }

  public void fill(Color color) {
    for (int i = 0; i < ledCount; i++) {
      mainBuffer.setLED(i, color);
    }
    this.setData(mainBuffer);
  }

  public void startRainbow() {
    changeLoopTo(this::rainbowLoop);
  }

  public void animateColor(Color color, AnimationType animation) {
    animateColor(color, animation, (ledCount * 2) / 3);
  }

  public void animateColor(Color color, AnimationType animation, int onLEDCount) {
    switch (animation) {
      case LEFT_TO_RIGHT:
        changeLoopTo(createLeftToRightAnimationLoop(color, onLEDCount));
        break;
      case RIGHT_TO_LEFT:
        changeLoopTo(createRightToLeftAnimationLoop(color, onLEDCount));
        break;
    }
  }

  public void stopAnimation() {
    changeLoopTo(() -> {}); // Empty runnable
  }

  public void blink(Color color, int blinkCount) {
    changeLoopTo(
        () -> {
          for (int i = 0; i < blinkCount; i++) {
            fill(color);
            Timer.delay(0.5);
            fill(Color.kBlack);
            Timer.delay(0.5);
          }
        });
  }

  /**
   * Breathe the led, this will light up the led and then light it down. This will repeat forever.
   *
   * @param color The color to breathe. see: WPILIB {@link Color}
   * @param delayTime The time delay between all the steps in seconds; NOT the total time!
   * @param max The max value of the led, normally the maximum is 255.
   * @param min The min value of the led, normally the minimum is 0.
   */
  public void breathe(Color color, double delayTime, int max, int min) {
    changeLoopTo(
        () -> {
          // Light UP The led
          for (int i = min; i < max; i++) {
            fill(new Color(color.red * i, color.green * i, color.blue * i));
            Timer.delay(delayTime);
          }
          // Light DOWN the led
          for (int i = max; i > min; i--) {
            fill(new Color(color.red * i, color.green * i, color.blue * i));
            Timer.delay(delayTime);
          }
        });
  }

  public void breathe(Color color, double delayTime) {
    breathe(color, delayTime, 255, 0);
  }

  public void breathe(Color color) {
    breathe(color, 0.01);
  }

  /**
   * Adds a runnable to the call list, this will be called every loop. You probably don't need to
   * use this, but it's here if you need it.
   *
   * @param func The runnable to add to the call list.
   */
  public void addRunnableToLoop(Runnable func) {
    callList.add(func);
  }

  @Override
  public void setLength(int length) {
    mainBuffer = new AddressableLEDBuffer(length);
    ledCount = length;
    super.setLength(length);
  }

  private void changeLoopTo(Runnable func) {
    callList.clear(); // Clear the list first
    callList.add(func); // then add the runnable
  }

  private void periodic() {
    // Call any functions in the callList
    for (var callable : callList) {
      callable.run();
    }
  }

  private void rainbowLoop() {
    for (int i = 0; i < ledCount; i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (rainbowFirstPixelHue + (i * 180 / ledCount)) % 80;
      // Set the value
      mainBuffer.setHSV(i, hue, 255, 255);
    }
    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
    this.setData(mainBuffer);
  }

  private Runnable createLeftToRightAnimationLoop(Color color, int onLEDCount) {
    return () -> {
      final int start = currentAnimationIndex > onLEDCount ? currentAnimationIndex - onLEDCount : 0;
      for (int i = start; i < ledCount && i < currentAnimationIndex; i++) {
        mainBuffer.setLED(i, color);
      }
      currentAnimationIndex++;
      currentAnimationIndex %= ledCount;
      this.setData(mainBuffer);
      Timer.delay(0.1);
    };
  }

  private Runnable createRightToLeftAnimationLoop(Color color, int onLEDCount) {
    return () -> {
      final int start =
          currentAnimationIndex < onLEDCount ? currentAnimationIndex + onLEDCount : ledCount;
      for (int i = start; i > 0 && i > currentAnimationIndex; i--) {
        mainBuffer.setLED(i, color);
      }
      currentAnimationIndex--;
      currentAnimationIndex =
          Math.floorMod(currentAnimationIndex, ledCount); // ! DO NOT USE % WITH NEGATIVE NUMBERS.
      this.setData(mainBuffer);
      Timer.delay(0.1);
    };
  }
}
