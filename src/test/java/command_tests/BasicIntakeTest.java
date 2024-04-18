package command_tests;

import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static utils.LEDTestUtils.checkForColorInAll;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.BasicIntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSystem;
import frc.utils.sim_utils.SparkMAXSimAddon;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class BasicIntakeTest {
  private IntakeSubsystem intakeSubsystem;
  private CANSparkMax intakeMotor;
  private LEDSubsystem ledSubsystem;
  private BasicIntakeCommand intakeCommand;
  private CommandScheduler commandScheduler;

  @BeforeEach
  public void setUp() {
    intakeSubsystem = new IntakeSubsystem();
    intakeMotor = SparkMAXSimAddon.getSparkMAX(IntakeConstants.kArmIntakeMotorCanID);
    ledSubsystem = LEDSystem.getInstance();
    intakeCommand = new BasicIntakeCommand(intakeSubsystem);

    // Enable robot for commands to run
    DriverStationSim.setEnabled(true);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData(); // ! Breaks without this
    commandScheduler = CommandScheduler.getInstance();
    commandScheduler.schedule(intakeCommand);
  }

  @AfterEach
  public void tearDown() {
    LEDSystem.resetLEDSubsystem();
    intakeSubsystem.close();
    intakeCommand.cancel();
    commandScheduler.close();
  }

  @Test
  void testSpeed() {
    commandScheduler.run();
    // Motor shouldn't be 0 when intaking
    assertNotEquals(0, intakeMotor.get());
  }

  @Test
  void testLEDBlinking() {
    commandScheduler.run();
    Timer.delay(0.2); // let led loop do its thing
    checkForColorInAll(ledSubsystem, Color.kRed, "Color should be red when started");
    Timer.delay(0.2);
    checkForColorInAll(ledSubsystem, Color.kBlack, "Color should be closed when blinking red");
  }
}
