package command_tests;

import static utils.LEDTestUtils.checkForColorInAll;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.led_commands.LEDIdleCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSystem;
import frc.utils.sim_utils.ColorSensorV3Wrapped;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class LEDIdleCommandTest {
  private LEDSubsystem ledSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private CommandScheduler commandScheduler;

  @BeforeEach
  public void setUp() {
    HAL.initialize(500, 0);
    ledSubsystem = LEDSystem.getInstance();
    intakeSubsystem = new IntakeSubsystem();
    commandScheduler = CommandScheduler.getInstance();
  }

  @AfterEach
  public void tearDown() {
    intakeSubsystem.close();
    LEDSystem.resetLEDSubsystem(); // closes and re init's ledSubsystem
    commandScheduler.close();
  }

  @Test
  void colorOnDisabled() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData(); // ! Breaks without this
    commandScheduler.schedule(
        new LEDIdleCommand(ledSubsystem, intakeSubsystem).ignoringDisable(true));
    commandScheduler.run();
    Timer.delay(0.1); // let led thread do its thing
    checkForColorInAll(ledSubsystem, Color.kOrangeRed, "Color Should be orange when disabled");
  }

  @Test
  void colorOnEnabled() {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData(); // ! Breaks without this
    commandScheduler.schedule(
        new LEDIdleCommand(ledSubsystem, intakeSubsystem).ignoringDisable(true));

    // * check without Note
    ColorSensorV3Wrapped.setRGBD(0, 0, 0, 0);
    commandScheduler.run();
    Timer.delay(0.1); // let led thread do its thing
    checkForColorInAll(ledSubsystem, Color.kRed, "Color should be red when no Note is detected");

    // * check with Note
    ColorSensorV3Wrapped.setRGBD(2500, 0, 0, 900);
    commandScheduler.run();
    Timer.delay(0.1); // let led thread do its thing
    checkForColorInAll(ledSubsystem, Color.kGreen, "Color should be green when Note is detected");
  }
}
