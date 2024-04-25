package command_tests.controller_commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import command_tests.utils.CommandTestBase;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import frc.robot.commands.VibrateControllerCommand;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class VibrateControllerCommandTest extends CommandTestBase {
  private VibrateControllerCommand vibrateControllerCommand;
  private XboxController controller;
  private XboxControllerSim controllerSim;

  private static final double kDelta = 0.01;

  private static final int kRepetitions = 3;
  private static final double kIntensity = 0.5; // 0.5 due to a bug in the simulator
  private static final double kWaitTime = 1.5;

  @BeforeEach
  public void setUp() {
    super.setUp();

    controller = new XboxController(1);
    controllerSim = new XboxControllerSim(1);
    vibrateControllerCommand =
        new VibrateControllerCommand(controller, kRepetitions, kIntensity, kWaitTime);
    commandScheduler.schedule(vibrateControllerCommand);
  }

  @AfterEach
  public void tearDown() {
    super.tearDown();
  }

  @Test
  void testItVibrates() {
    commandScheduler.run();
    assertEquals(kIntensity, controllerSim.getRumble(RumbleType.kLeftRumble), kDelta);
  }

  @Test
  void testItStopsVibrating() {
    commandScheduler.run();
    Timer.delay(kWaitTime);
    commandScheduler.run();
    assertEquals(0, controllerSim.getRumble(RumbleType.kLeftRumble), kDelta);
  }

  @Test
  void testItRepeats() {
    // run command scheduler twice to end wait command and start vibration
    for (int i = 0; i < kRepetitions; i++) {
      commandScheduler.run();
      commandScheduler.run();
      assertEquals(
          kIntensity,
          controllerSim.getRumble(RumbleType.kLeftRumble),
          kDelta,
          "controller should vibrate at index: " + i);
      Timer.delay(kWaitTime + 0.1);
      commandScheduler.run();
      commandScheduler.run();
      assertEquals(
          0,
          controllerSim.getRumble(RumbleType.kLeftRumble),
          kDelta,
          "controller should stop vibrating at index: " + i);
      Timer.delay(kWaitTime + 0.1);
      commandScheduler.run();
      commandScheduler.run();
    }
  }
}
